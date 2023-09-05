// THERMOSTAT HY02B05 TYUA PROTOCOL VERSION 1
#include "esphome.h"
using namespace esphome;

#ifdef USE_CAPTIVE_PORTAL
 #include "esphome/components/captive_portal/captive_portal.h"
#endif

struct ApiCommandTuya {
      uint8_t datapointapi;
      uint32_t valueapi;
    };
    
ApiCommandTuya sending {}; 
optional<uint8_t> reception {};
optional<ApiCommandTuya> sending_switch {};
optional<ApiCommandTuya> sending_enum {};
optional<ApiCommandTuya> sending_sensor {};
optional<uint8_t> sending_sensor_raw {};
optional<uint32_t> last_blocking_lock {};

static const char *const TAG = "tuya";
static const int COMMAND_DELAY = 10;
static const int RECEIVE_TIMEOUT = 300;
static const int MAX_RETRIES = 5;
static const int DELAY_REC_MODE_BLOC_LOCK = 30000;
  
class TuComponent : public Component, public UARTDevice {
  public:
  
    enum class TuyaDatapointType : uint8_t {
      RAW = 0x00,      // variable length
      BOOLEAN = 0x01,  // 1 byte (0/1)
      INTEGER = 0x02,  // 4 byte
      STRING = 0x03,   // variable length
      ENUM = 0x04,     // 1 byte
      BITMASK = 0x05,  // 1/2/4 bytes
    };
 
    struct TuyaDatapoint {
      uint8_t id;
      TuyaDatapointType type;
      size_t len;
      union {
        bool value_bool;
        int value_int;
        uint32_t value_uint;
        uint8_t value_enum;
        uint32_t value_bitmask;
      };
      std::string value_string;
      std::vector<uint8_t> value_raw;
    };
 
    enum class TuyaCommandType : uint8_t {
      HEARTBEAT = 0x00,
      PRODUCT_QUERY = 0x01,
      CONF_QUERY = 0x02,
      WIFI_STATE = 0x03,
      WIFI_RESET = 0x04,
      WIFI_SELECT = 0x05,
      DATAPOINT_DELIVER = 0x06,
      DATAPOINT_REPORT = 0x07,
      DATAPOINT_QUERY = 0x08,
      WIFI_TEST = 0x0E,
      LOCAL_TIME_QUERY = 0x1C,
      WIFI_RSSI = 0x24,
      GET_NETWORK_STATUS = 0x2B,
    };
 
    enum class TuyaInitState : uint8_t {
      INIT_HEARTBEAT = 0x00,
      INIT_PRODUCT,
      INIT_CONF,
      INIT_WIFI,
      INIT_DATAPOINT,
      INIT_DONE,
    };
 
    struct TuyaCommand {
      TuyaCommandType cmd;
      std::vector<uint8_t> payload;
    };  
  
 #ifdef USE_TIME
    void set_time_id(time::RealTimeClock *time_id) { this->time_id_ = time_id; }
 #endif
 
    TuComponent(UARTComponent *parent) : UARTDevice(parent) {} 
 
    void setup() override {
      set_time_id(homeassistant_time);    
      this->send_command_(TuyaCommand{.cmd = TuyaCommandType::WIFI_STATE, .payload = std::vector<uint8_t>{0x01}});
    } 
 
    void loop() override {
      while (this->available()) {
        uint8_t c;
        this->read_byte(&c);
        this->handle_char_(c);
      }
      process_command_queue_();
    }
 
    void dump_config() override {
      ESP_LOGCONFIG(TAG, "Tuya:");
      if (this->init_state_ != TuyaInitState::INIT_DONE) {
        if (this->init_failed_) {
          ESP_LOGCONFIG(TAG, "  Initialization failed. Current init_state: %u", static_cast<uint8_t>(this->init_state_));
        } else {
          ESP_LOGCONFIG(TAG, "  Configuration will be reported when setup is complete. Current init_state: %u",
          static_cast<uint8_t>(this->init_state_));
        }
        ESP_LOGCONFIG(TAG, "  If no further output is received, confirm that this is a supported Tuya device.");
        return;
      }
      for (auto &info : this->datapoints_) {
        if (info.type == TuyaDatapointType::RAW) {
          ESP_LOGCONFIG(TAG, "  Datapoint %u: raw (value: %s)", info.id, format_hex_pretty(info.value_raw).c_str());
        } else if (info.type == TuyaDatapointType::BOOLEAN) {
          ESP_LOGCONFIG(TAG, "  Datapoint %u: switch (value: %s)", info.id, ONOFF(info.value_bool));
        } else if (info.type == TuyaDatapointType::INTEGER) {
          ESP_LOGCONFIG(TAG, "  Datapoint %u: int value (value: %d)", info.id, info.value_int);   
        } else if (info.type == TuyaDatapointType::STRING) { 
          ESP_LOGCONFIG(TAG, "  Datapoint %u: string value (value: %s)", info.id, info.value_string.c_str());
        } else if (info.type == TuyaDatapointType::ENUM) {
          ESP_LOGCONFIG(TAG, "  Datapoint %u: enum (value: %d)", info.id, info.value_enum);
        } else if (info.type == TuyaDatapointType::BITMASK) {
          ESP_LOGCONFIG(TAG, "  Datapoint %u: bitmask (value: %x)", info.id, info.value_bitmask);
        } else {
          ESP_LOGCONFIG(TAG, "  Datapoint %u: unknown", info.id);
        }
      }
      ESP_LOGCONFIG(TAG, "  Product: '%s'", this->product_.c_str());
     } 
     
    protected:
      uint32_t last_command_timestamp_ = 0;
      uint32_t last_rx_char_timestamp_ = 0;
      std::string product_ = "";  
      int init_retries_{0};
      bool init_failed_{false};
      std::vector<TuyaDatapoint> datapoints_;
      std::vector<uint8_t> rx_message_;
      std::vector<TuyaCommand> command_queue_;
      optional<TuyaCommandType> expected_response_{};
      optional<uint32_t> last_press_mode_ {};
      uint8_t protocol_version_ = -1;
      TuyaInitState init_state_ = TuyaInitState::INIT_HEARTBEAT;
      uint8_t wifi_status_ = -1;
      std::vector<uint8_t> ignore_mcu_update_on_datapoints_{101, 108};
      std::vector<uint8_t> value_points_ {};
      std::vector<uint8_t> value_points_1_ {};
      int target_temp_ {};
  #ifdef USE_TIME
      optional<time::RealTimeClock *> time_id_{};
  #endif
 
      void handle_char_(uint8_t c) { 
        this->rx_message_.push_back(c);             
        if (!this->validate_message_()) {
          this->rx_message_.clear();
        } else {
          last_rx_char_timestamp_ = millis();
        }
      }  
    
      bool validate_message_() {
        uint32_t at = this->rx_message_.size() - 1;
        auto *data = &this->rx_message_[0];
        uint8_t new_byte = data[at];
 
        // Byte 0: HEADER1 (always 0x55)
        if (at == 0)
          return new_byte == 0x55;
        // Byte 1: HEADER2 (always 0xAA)
        if (at == 1)
          return new_byte == 0xAA;
 
        // Byte 2: VERSION
        // no validation for the following fields:
        uint8_t version = data[2];
        if (at == 2)
          return true;
        // Byte 3: COMMAND
        uint8_t command = data[3]; 
        if (at == 3)
          return true;
 
        // Byte 4: LENGTH1
        // Byte 5: LENGTH2
        if (at <= 5) {
          // no validation for these fields
          return true;
        }
 
        uint16_t length = (uint16_t(data[4]) << 8) | (uint16_t(data[5]));
 
        // wait until all data is read
        if (at - 6 < length)
          return true;
 
        // Byte 6+LEN: CHECKSUM - sum of all bytes (including header) modulo 256
        uint8_t rx_checksum = new_byte;
        uint8_t calc_checksum = 0;
        for (uint32_t i = 0; i < 6 + length; i++)
          calc_checksum += data[i];
        
     
        if (rx_checksum != calc_checksum) {
          ESP_LOGW(TAG, "Tuya Received invalid message checksum %02X!=%02X", rx_checksum, calc_checksum);
          return false;
        }
 
        // valid message
        const uint8_t *message_data = data + 6;
        ESP_LOGV(TAG, "Received Tuya: CMD=0x%02X VERSION=%u DATA=[%s] INIT_STATE=%u", command, version, format_hex_pretty(message_data, length).c_str(), static_cast<uint8_t>(this->init_state_));
      
        this->handle_command_(command, version, message_data, length);

     
        // return false to reset rx buffer
        return false;
      } 
    
      void handle_command_(uint8_t command, uint8_t version, const uint8_t *buffer, size_t len) {
        TuyaCommandType command_type = (TuyaCommandType) command;
      
        if (this->expected_response_.has_value() && this->expected_response_ == command_type) {
          this->expected_response_.reset();
          this->command_queue_.erase(command_queue_.begin());
          this->init_retries_ = 0;
        }
 
        switch (command_type) {
          case TuyaCommandType::HEARTBEAT:
            ESP_LOGV(TAG, "MCU Heartbeat (0x%02X)", buffer[0]);
            this->protocol_version_ = version;
            if (buffer[0] == 0) {
              ESP_LOGI(TAG, "MCU restarted");
              this->init_state_ = TuyaInitState::INIT_HEARTBEAT;
            }
            if (this->init_state_ == TuyaInitState::INIT_HEARTBEAT) {
              this->init_state_ = TuyaInitState::INIT_PRODUCT;
              this->send_empty_command_(TuyaCommandType::PRODUCT_QUERY);
            }
            if (this->init_state_ == TuyaInitState::INIT_DONE) {
              this->send_empty_command_(TuyaCommandType::DATAPOINT_QUERY);
            }
            break;
          case TuyaCommandType::PRODUCT_QUERY: {
            // check it is a valid string made up of printable characters
            bool valid = true;
            for (size_t i = 0; i < len; i++) {
              if (!std::isprint(buffer[i])) {
                valid = false;
                break;
              }
            }
            if (valid) {
              this->product_ = std::string(reinterpret_cast<const char *>(buffer), len);
            } else {
              this->product_ = R"({"p":"INVALID"})";
            }
            if (this->init_state_ == TuyaInitState::INIT_PRODUCT) {
              this->init_state_ = TuyaInitState::INIT_CONF;
              this->send_empty_command_(TuyaCommandType::CONF_QUERY);
            }
            break;
          }
          case TuyaCommandType::CONF_QUERY: {
            this->init_state_ = TuyaInitState::INIT_WIFI;
            ESP_LOGV(TAG, "Configured WIFI_STATE periodic send");
            this->set_interval("wifi", 1000, [this] { this->send_wifi_status_(); });
            break;
          }
          case TuyaCommandType::WIFI_STATE:
           // if (this->init_state_ == TuyaInitState::INIT_WIFI) {
             // this->init_state_ = TuyaInitState::INIT_DATAPOINT;
              //this->send_empty_command_(TuyaCommandType::DATAPOINT_QUERY);
            //}
            break;
          case TuyaCommandType::WIFI_RESET:
            ESP_LOGE(TAG, "WIFI_RESET is not handled");
            break;
          case TuyaCommandType::WIFI_SELECT:
            ESP_LOGE(TAG, "WIFI_SELECT is not handled");
            break;
          case TuyaCommandType::DATAPOINT_DELIVER:
            break;
          case TuyaCommandType::DATAPOINT_REPORT:
            this->handle_datapoints_(buffer, len);
            break;
          case TuyaCommandType::DATAPOINT_QUERY:
            break;
          case TuyaCommandType::WIFI_TEST:
            this->send_command_(TuyaCommand{.cmd = TuyaCommandType::WIFI_TEST, .payload = std::vector<uint8_t>{0x00, 0x00}});
            break;
          case TuyaCommandType::WIFI_RSSI:
            this->send_command_(TuyaCommand{.cmd = TuyaCommandType::WIFI_RSSI, .payload = std::vector<uint8_t>{get_wifi_rssi_()}});
            break;
          case TuyaCommandType::LOCAL_TIME_QUERY:
        #ifdef USE_TIME
            if (this->time_id_.has_value()) {
              this->send_local_time_();
              auto *time_id = *this->time_id_;
            } else {
              ESP_LOGW(TAG, "LOCAL_TIME_QUERY is not handled because time is not configured");
            }
        #else
           ESP_LOGE(TAG, "LOCAL_TIME_QUERY is not handled");
        #endif 
            break;
          case TuyaCommandType::GET_NETWORK_STATUS: {
            uint8_t wifi_status = this->get_wifi_status_code_();
            this->send_command_(
            TuyaCommand{.cmd = TuyaCommandType::GET_NETWORK_STATUS, .payload = std::vector<uint8_t>{wifi_status}});
            ESP_LOGV(TAG, "Network status requested, reported as %i", wifi_status);
            break;
          }
          default:
            ESP_LOGE(TAG, "Invalid command (0x%02X) received", command);
        }
      } 
    
      void process_command_queue_() {
        uint32_t now = millis();
        uint32_t delay = now - this->last_command_timestamp_;
        
        if (now - this->last_rx_char_timestamp_ > RECEIVE_TIMEOUT) {
          this->rx_message_.clear();
        }
 
        if (this->expected_response_.has_value() && delay > RECEIVE_TIMEOUT) {
          this->expected_response_.reset();
          if (init_state_ != TuyaInitState::INIT_DONE) {
            if (++this->init_retries_ >= MAX_RETRIES) {
              this->init_failed_ = true;
              ESP_LOGE(TAG, "Initialization failed at init_state %u", static_cast<uint8_t>(this->init_state_));
              this->command_queue_.erase(command_queue_.begin());
              this->init_retries_ = 0;
            }
          } else {
            this->command_queue_.erase(command_queue_.begin());
          }
        }
        
        if (!id(lock).state && last_blocking_lock.has_value()  && ((now - last_blocking_lock.value()) > DELAY_REC_MODE_BLOC_LOCK)) {
          last_blocking_lock.reset();
          id(lock).turn_on();
        }
        
        if (this->last_press_mode_.has_value() && ((now - this->last_press_mode_.value()) > DELAY_REC_MODE_BLOC_LOCK)) {
          this->last_press_mode_.reset();
          if (id(options_mode).state != "TempProg") {
            auto index = id(options_mode).active_index();
            if (id(my_global_mode) != index.value()) {
              id(my_global_mode) = index.value();   
            }
          }    
        }
          
  
        if (sending_switch.has_value()) {
          sending_switch.reset();
          if (sending_switch->datapointapi == 1 || sending_switch->datapointapi == 6 || \
             ((id(power).state && !id(lock).state) && (id(setting).state && ((sending_switch->datapointapi == 106) || \
             (sending_switch->datapointapi == 107))))) {  
            this->set_numeric_datapoint_value_(sending_switch->datapointapi, TuyaDatapointType::BOOLEAN, sending_switch->valueapi, 1, false);
            if (((sending_switch->datapointapi == 1) && !sending_switch->valueapi) || ((sending_switch->datapointapi == 6) && sending_switch->valueapi)) {
              if (id(setting).state) {
                id(setting).turn_off();
              }
            }
          } else {
            optional<TuyaDatapoint> datapoint_p {}; 
            if ((sending_switch->datapointapi == 5) || (sending_switch->datapointapi == 7) || (sending_switch->datapointapi == 8) || (sending_switch->datapointapi == 9)) {
              if (sending_switch->datapointapi == 5) {
                if (id(setting_points).state) {
                  id(setting_points).turn_off(); 
                } 
              }  
              if (sending_switch->datapointapi == 7) {
                if (id(cansel).state) {
                  id(cansel).turn_off();
                }              
                if (id(wor_points).state) {
                  id(wor_points).turn_off();
                }  
                if (id(week_points).state) {
                  id(week_points).turn_off();
                } 
              }    
              if ((sending_switch->datapointapi == 8) || (sending_switch->datapointapi == 9)) {
                if (sending_switch->datapointapi == 8) {
                  datapoint_p = this->get_datapoint_(119);              
                } else {
                  datapoint_p = this->get_datapoint_(121);
                }  
                reception = 119;
                id(number_hour_1).publish_state(datapoint_p->value_raw[0] & 0x7F);
                id(number_minutes_1).publish_state(datapoint_p->value_raw[1] & 0x7F);
                id(number_temperature_1).publish_state(datapoint_p->value_raw[2] & 0x7F);
                id(number_hour_2).publish_state(datapoint_p->value_raw[3] & 0x7F);
                id(number_minutes_2).publish_state(datapoint_p->value_raw[4] & 0x7F);
                id(number_temperature_2).publish_state(datapoint_p->value_raw[5] & 0x7F);
                id(number_hour_3).publish_state(datapoint_p->value_raw[6] & 0x7F);
                id(number_minutes_3).publish_state(datapoint_p->value_raw[7] & 0x7F);
                id(number_temperature_3).publish_state(datapoint_p->value_raw[8] & 0x7F); 
                if (sending_switch->datapointapi == 9) {
                  datapoint_p = this->get_datapoint_(122);              
                } else {
                  datapoint_p = this->get_datapoint_(120);
                }  
                id(number_hour_4).publish_state(datapoint_p->value_raw[0] & 0x7F);
                id(number_minutes_4).publish_state(datapoint_p->value_raw[1] & 0x7F);
                id(number_temperature_4).publish_state(datapoint_p->value_raw[2] & 0x7F);
                id(number_hour_5).publish_state(datapoint_p->value_raw[3] & 0x7F);
                id(number_minutes_5).publish_state(datapoint_p->value_raw[4] & 0x7F);
                id(number_temperature_5).publish_state(datapoint_p->value_raw[5] & 0x7F);
                id(number_hour_6).publish_state(datapoint_p->value_raw[6] & 0x7F);
                id(number_minutes_6).publish_state(datapoint_p->value_raw[7] & 0x7F);
                id(number_temperature_6).publish_state(datapoint_p->value_raw[8] & 0x7F);  
                reception.reset(); 
              }
            } else {
              datapoint_p = this->get_datapoint_(sending_switch->datapointapi); 
              reception = datapoint_p->id;
              this->set_numeric_datapoint_value_(datapoint_p->id, TuyaDatapointType::BOOLEAN, datapoint_p->value_bool, 1, true);
            }
          }    
        } 
          
        if (sending_enum.has_value()) {
          sending_enum.reset();
          optional<TuyaDatapoint> datapoint_p {}; 
          datapoint_p = this->get_datapoint_(sending_enum->datapointapi); 
          if (id(power).state && !id(lock).state && \
             ((!id(setting).state && (sending_enum->datapointapi == 4)) || (id(setting).state && !(sending_enum->datapointapi == 4)))) {
            if (sending_enum->datapointapi == 4) {
              if ((sending_enum->valueapi == 0) && (datapoint_p->value_enum == 0)) {
                sending.valueapi = 1;
                sending_enum = sending;
                sending_enum.reset();
              }
            }    
            this->set_numeric_datapoint_value_(sending_enum->datapointapi, TuyaDatapointType::ENUM, sending_enum->valueapi, 1, false);
          } else {
            reception = datapoint_p->id;
            this->set_numeric_datapoint_value_(datapoint_p->id, TuyaDatapointType::ENUM, datapoint_p->value_enum, 1, true);
          }    
        } 
        
        if (sending_sensor.has_value()) {
          sending_sensor.reset();
          if ((id(power).state && !id(lock).state) && \
             ((!id(setting).state && ((sending_sensor->datapointapi == 104) || (sending_sensor->datapointapi == 105))) || \
              (id(setting).state && !((sending_sensor->datapointapi == 104) || (sending_sensor->datapointapi == 105))) || \
              (sending_sensor->datapointapi == 2))) {
            this->set_numeric_datapoint_value_(sending_sensor->datapointapi, TuyaDatapointType::INTEGER, sending_sensor->valueapi, 4, false);
          } else {
            optional<TuyaDatapoint> datapoint_p {}; 
            datapoint_p = this->get_datapoint_(sending_sensor->datapointapi); 
            reception = datapoint_p->id;
            this->set_numeric_datapoint_value_(datapoint_p->id, TuyaDatapointType::INTEGER, datapoint_p->value_int, 4, true);
          }
        }
        
        if (sending_sensor_raw.has_value() && id(cansel).state) {
          sending_sensor_raw.reset();
          optional<TuyaDatapoint> datapoint_p {}; 
          optional<TuyaDatapoint> datapoint_p_1 {};               
          if (id(wor_points).state) {
            datapoint_p = this->get_datapoint_(119);
            datapoint_p_1 = this->get_datapoint_(120);
          }
          if (id(week_points).state) {
            datapoint_p = this->get_datapoint_(121);
            datapoint_p_1 = this->get_datapoint_(122);
          }
          
          
          last_blocking_lock = millis();
          reception = 119;  
          switch (sending_sensor_raw.value()) {
            case 0:
              if ((id(number_hour_1).state * 60 + id(number_minutes_1).state) >= (id(number_hour_2).state * 60 + id(number_minutes_2).state))
                id(number_hour_1).publish_state(datapoint_p->value_raw[0] & 0x7F);
              break;
            case 1:
              if ((id(number_hour_1).state * 60 + id(number_minutes_1).state) >= (id(number_hour_2).state * 60 + id(number_minutes_2).state))
                id(number_minutes_1).publish_state(datapoint_p->value_raw[1] & 0x7F); 
              break;
            case 2:
              if (id(number_temperature_1).state > id(number_max_temperature).state) 
                id(number_temperature_1).publish_state(id(number_max_temperature).state); 
              if (id(number_temperature_1).state < id(number_min_temperature).state)
                id(number_temperature_1).publish_state(id(number_min_temperature).state);
              break;
            case 3:
              if (((id(number_hour_2).state * 60 + id(number_minutes_2).state) <= (id(number_hour_1).state * 60 + id(number_minutes_1).state)) || \
                  ((id(number_hour_2).state * 60 + id(number_minutes_2).state) >= (id(number_hour_3).state * 60 + id(number_minutes_3).state)))
                id(number_hour_2).publish_state(datapoint_p->value_raw[3] & 0x7F); 
              break;
            case 4:
              if (((id(number_hour_2).state * 60 + id(number_minutes_2).state) <= (id(number_hour_1).state * 60 + id(number_minutes_1).state)) || \
                  ((id(number_hour_2).state * 60 + id(number_minutes_2).state) >= (id(number_hour_3).state * 60 + id(number_minutes_3).state)))               
                id(number_minutes_2).publish_state(datapoint_p->value_raw[4] & 0x7F);
              break;
            case 5:  
              if (id(number_temperature_2).state > id(number_max_temperature).state) 
                id(number_temperature_2).publish_state(id(number_max_temperature).state); 
              if (id(number_temperature_2).state < id(number_min_temperature).state)
                id(number_temperature_2).publish_state(id(number_min_temperature).state); 
              break;  
            case 6:
              if (((id(number_hour_3).state * 60 + id(number_minutes_3).state) <= (id(number_hour_2).state * 60 + id(number_minutes_2).state)) || \
                  ((id(number_hour_3).state * 60 + id(number_minutes_3).state) >= (id(number_hour_4).state * 60 + id(number_minutes_4).state)))
                id(number_hour_3).publish_state(datapoint_p->value_raw[6] & 0x7F);  
              break;
            case 7:
              if (((id(number_hour_3).state * 60 + id(number_minutes_3).state) <= (id(number_hour_2).state * 60 + id(number_minutes_2).state)) || \
                  ((id(number_hour_3).state * 60 + id(number_minutes_3).state) >= (id(number_hour_4).state * 60 + id(number_minutes_4).state)))               
                id(number_minutes_3).publish_state(datapoint_p->value_raw[7] & 0x7F);  
              break;
            case 8:  
              if (id(number_temperature_3).state > id(number_max_temperature).state) 
                id(number_temperature_3).publish_state(id(number_max_temperature).state); 
              if (id(number_temperature_3).state < id(number_min_temperature).state)
                id(number_temperature_3).publish_state(id(number_min_temperature).state); 
              break;                
            case 9:
              if (((id(number_hour_4).state * 60 + id(number_minutes_4).state) <= (id(number_hour_3).state * 60 + id(number_minutes_3).state)) || \
                  ((id(number_hour_4).state * 60 + id(number_minutes_4).state) >= (id(number_hour_5).state * 60 + id(number_minutes_5).state)))
                id(number_hour_4).publish_state(datapoint_p_1->value_raw[0] & 0x7F);   
              break;
            case 10:
              if (((id(number_hour_4).state * 60 + id(number_minutes_4).state) <= (id(number_hour_3).state * 60 + id(number_minutes_3).state)) || \
                  ((id(number_hour_4).state * 60 + id(number_minutes_4).state) >= (id(number_hour_5).state * 60 + id(number_minutes_5).state)))               
                id(number_minutes_4).publish_state(datapoint_p_1->value_raw[1] & 0x7F); 
              break;
            case 11:
              if (id(number_temperature_4).state > id(number_max_temperature).state) 
                id(number_temperature_4).publish_state(id(number_max_temperature).state); 
              if (id(number_temperature_4).state < id(number_min_temperature).state)
                id(number_temperature_4).publish_state(id(number_min_temperature).state);  
              break;
            case 12:
              if (((id(number_hour_5).state * 60 + id(number_minutes_5).state) <= (id(number_hour_4).state * 60 + id(number_minutes_4).state)) || \
                  ((id(number_hour_5).state * 60 + id(number_minutes_5).state) >= (id(number_hour_6).state * 60 + id(number_minutes_6).state)))
                id(number_hour_5).publish_state(datapoint_p_1->value_raw[3] & 0x7F); 
              break;
            case 13:
              if (((id(number_hour_5).state * 60 + id(number_minutes_5).state) <= (id(number_hour_4).state * 60 + id(number_minutes_4).state)) || \
                  ((id(number_hour_5).state * 60 + id(number_minutes_5).state) >= (id(number_hour_6).state * 60 + id(number_minutes_6).state)))               
                id(number_minutes_5).publish_state(datapoint_p_1->value_raw[4] & 0x7F);   
              break;
            case 14:  
              if (id(number_temperature_5).state > id(number_max_temperature).state) 
                id(number_temperature_5).publish_state(id(number_max_temperature).state); 
              if (id(number_temperature_5).state < id(number_min_temperature).state)
                id(number_temperature_5).publish_state(id(number_min_temperature).state);   
              break;  
            case 15:
              if ((id(number_hour_6).state * 60 + id(number_minutes_6).state) <= (id(number_hour_5).state * 60 + id(number_minutes_5).state))
                id(number_hour_6).publish_state(datapoint_p_1->value_raw[6] & 0x7F); 
              break;
            case 16:
              if ((id(number_hour_6).state * 60 + id(number_minutes_6).state) <= (id(number_hour_5).state * 60 + id(number_minutes_5).state))
                id(number_minutes_6).publish_state(datapoint_p_1->value_raw[7] & 0x7F);  
              break;
            case 17:  
              if (id(number_temperature_6).state > id(number_max_temperature).state) 
                id(number_temperature_6).publish_state(id(number_max_temperature).state); 
              if (id(number_temperature_6).state < id(number_min_temperature).state)
                id(number_temperature_6).publish_state(id(number_min_temperature).state); 
              break;                
            default:
              break;
          }             
          reception.reset(); 
            
          this->value_points_ = {(uint8_t)id(number_hour_1).state, (uint8_t)id(number_minutes_1).state, (uint8_t)id(number_temperature_1).state,
                                 (uint8_t)id(number_hour_2).state, (uint8_t)id(number_minutes_2).state, (uint8_t)id(number_temperature_2).state,
                                 (uint8_t)id(number_hour_3).state, (uint8_t)id(number_minutes_3).state, (uint8_t)id(number_temperature_3).state};             
            
          this->value_points_1_ = {(uint8_t)id(number_hour_4).state, (uint8_t)id(number_minutes_4).state, (uint8_t)id(number_temperature_4).state,
                                   (uint8_t)id(number_hour_5).state, (uint8_t)id(number_minutes_5).state, (uint8_t)id(number_temperature_5).state,
                                   (uint8_t)id(number_hour_6).state, (uint8_t)id(number_minutes_6).state, (uint8_t)id(number_temperature_6).state}; 
                                   
          if ((this->value_points_ == datapoint_p->value_raw) && (this->value_points_1_ == datapoint_p_1->value_raw)) {
            id(cansel).turn_off();
          }
          
          if(id(save).state) {
            this->set_raw_datapoint_value_(datapoint_p->id, this->value_points_, false);
            id(save).turn_off();
            this->set_raw_datapoint_value_(datapoint_p_1->id, this->value_points_1_, false);
            id(cansel).turn_off();
          }    
        } 
        
        // Left check of delay since last command in case there's ever a command sent by calling send_raw_command_ directly
        if (delay > COMMAND_DELAY && !this->command_queue_.empty() && this->rx_message_.empty() &&
          !this->expected_response_.has_value()) {
          this->send_raw_command_(command_queue_.front());
          if (!this->expected_response_.has_value())
            this->command_queue_.erase(command_queue_.begin());
          }
      }  
 
 
      void send_command_(const TuyaCommand &command) {
        command_queue_.push_back(command);
        process_command_queue_();
      }
 
      void send_empty_command_(TuyaCommandType command) {
        send_command_(TuyaCommand{.cmd = command, .payload = std::vector<uint8_t>{}});
      } 
 
      void send_raw_command_(TuyaCommand command) {
        uint8_t len_hi = (uint8_t)(command.payload.size() >> 8);
        uint8_t len_lo = (uint8_t)(command.payload.size() & 0xFF);
        uint8_t version = 0;
 
        this->last_command_timestamp_ = millis();
        switch (command.cmd) {
          case TuyaCommandType::HEARTBEAT:
            this->expected_response_ = TuyaCommandType::HEARTBEAT;
            break;
          case TuyaCommandType::PRODUCT_QUERY:
            this->expected_response_ = TuyaCommandType::PRODUCT_QUERY;
            break;
          case TuyaCommandType::CONF_QUERY:
            this->expected_response_ = TuyaCommandType::CONF_QUERY;
            break;
          case TuyaCommandType::DATAPOINT_DELIVER:
          case TuyaCommandType::DATAPOINT_QUERY:
            this->expected_response_ = TuyaCommandType::DATAPOINT_REPORT;
            break;
          default:
            break;
        }
 
        ESP_LOGV(TAG, "Sending Tuya: CMD=0x%02X VERSION=%u DATA=[%s] INIT_STATE=%u", static_cast<uint8_t>(command.cmd), version, format_hex_pretty(command.payload).c_str(), static_cast<uint8_t>(this->init_state_));
 
        this->write_array({0x55, 0xAA, version, (uint8_t) command.cmd, len_hi, len_lo});
        if (!command.payload.empty())
          this->write_array(command.payload.data(), command.payload.size());
 
        uint8_t checksum = 0x55 + 0xAA + (uint8_t) command.cmd + len_hi + len_lo;
        for (auto &data : command.payload)
          checksum += data;
        this->write_byte(checksum);
      } 
 
      void handle_datapoints_(const uint8_t *buffer, size_t len) {
        optional<TuyaDatapoint> datapoint_p {};    
        while (len >= 4) {
          TuyaDatapoint datapoint{};
          datapoint.id = buffer[0];
          datapoint.type = (TuyaDatapointType) buffer[1];
          datapoint.value_uint = 0;

          size_t data_size = (buffer[2] << 8) + buffer[3];
          const uint8_t *data = buffer + 4;
          size_t data_len = len - 4;
          if (data_size > data_len) {
            ESP_LOGW(TAG, "Datapoint %u is truncated and cannot be parsed (%zu > %zu)", datapoint.id, data_size, data_len);
            return;
          }
       
          datapoint.len = data_size;
          len -= data_size + 4;
          buffer = data + data_size;
        
          // drop update if datapoint is in ignore_mcu_datapoint_update list
          bool skip = false; 
          for (auto i : this->ignore_mcu_update_on_datapoints_) {
            if (datapoint.id == i) {
              ESP_LOGV(TAG, "Datapoint %u found in ignore_mcu_update_on_datapoints list, dropping MCU update", datapoint.id);
              skip = true;
              break;
            }
          }
          if (skip) {
            continue;
          }    

          if ((this->init_state_ != TuyaInitState::INIT_DONE) && (this->init_state_ != TuyaInitState::INIT_DATAPOINT)) {
            //Last state record mode
            if (this->init_state_ == TuyaInitState::INIT_HEARTBEAT) {
              if (datapoint.id == 4) {
                //Last state record mode
                if (id(my_global_mode) != data[0]) {
                  datapoint.value_enum = data[0];
                  this->datapoints_.push_back(datapoint);  
                } else {
                  this->datapoints_.clear();
                  if (datapoint_p.has_value()) {
                    datapoint_p.reset(); 
                  } 
                }
              }
            }  
              
            // Activation of reading all data points 
            if (this->init_state_ == TuyaInitState::INIT_WIFI) {
              if (datapoint.id == 115) {
                datapoint_p = this->get_datapoint_(115);
                if (!datapoint_p.has_value()) {
                  datapoint.value_int = encode_uint32(data[0], data[1], data[2], data[3]);
                  this->datapoints_.push_back(datapoint); 
                } else {
                  this->datapoints_.clear();
                  datapoint_p.reset();
                  this->init_state_ = TuyaInitState::INIT_DATAPOINT;
                  this->send_empty_command_(TuyaCommandType::DATAPOINT_QUERY);
                }
              }
            }  
              
            if (datapoint.id == 118) {
              if (this->init_state_ == TuyaInitState::INIT_HEARTBEAT) {
                datapoint_p = this->get_datapoint_(4);
                if (datapoint_p.has_value()) {
                  this->set_numeric_datapoint_value_(4, TuyaDatapointType::ENUM, (uint8_t)id(my_global_mode), 1, false);       
                } else {
                  this->set_interval("heartbeat", 15000, [this] { this->send_empty_command_(TuyaCommandType::HEARTBEAT); });  
                }
              }    
              if (this->init_state_ == TuyaInitState::INIT_WIFI) {                
                datapoint_p = this->get_datapoint_(115);
                if (datapoint_p.has_value()) {
                  this->set_numeric_datapoint_value_(115, TuyaDatapointType::INTEGER, datapoint_p->value_int, 4, true);
                }  
              }
            }
            continue;
          }

          datapoint_p = this->get_datapoint_(datapoint.id);  
      
          switch (datapoint.type) {
            case TuyaDatapointType::RAW:
              datapoint.value_raw = std::vector<uint8_t>(data, data + data_size); 
              if ((datapoint_p->value_raw == datapoint.value_raw) && datapoint_p.has_value()) {continue;}
              ESP_LOGD(TAG, "Datapoint %u update to %s", datapoint.id, format_hex_pretty(datapoint.value_raw).c_str()); 
              break;
            case TuyaDatapointType::BOOLEAN:
              if (data_size != 1) {
                ESP_LOGW(TAG, "Datapoint %u has bad boolean len %zu", datapoint.id, data_size);
                return;
              }
              datapoint.value_bool = data[0];
              if (!reception.has_value()) {
                if ((datapoint_p->value_bool == datapoint.value_bool) && datapoint_p.has_value()) {
                  continue;
                } 
              } else {
                if (datapoint.id == reception.value()) {
                  reception.reset();
                }
              }
              ESP_LOGD(TAG, "Datapoint %u update to %d", datapoint.id, datapoint.value_bool); 
              if(datapoint.id == 1) {
                id(power).publish_state(datapoint.value_bool);   
                if(datapoint.value_bool) {
                  id(my_custom_climate).action = climate::CLIMATE_ACTION_IDLE;
                  id(my_custom_climate).mode = climate::CLIMATE_MODE_OFF;
                  id(my_custom_climate).target_temperature = target_temp_ / 10;
                } else {
                  id(my_custom_climate).target_temperature = 0.0f;
                  id(my_custom_climate).mode = climate::CLIMATE_MODE_OFF;
                  id(my_custom_climate).action = climate::CLIMATE_ACTION_OFF; 
                } 
                id(my_custom_climate).publish_state();
                break;
              }  
              if(datapoint.id == 6) {
                id(lock).publish_state(datapoint.value_bool);
                break;
              }
              if(datapoint.id == 102) {
                if (datapoint.value_bool) {
                  id(my_custom_climate).action = climate::CLIMATE_ACTION_HEATING;
                  id(my_custom_climate).mode = climate::CLIMATE_MODE_HEAT;  
                } else {
                  id(my_custom_climate).mode = climate::CLIMATE_MODE_OFF;
                  if (id(power).state) {
                    id(my_custom_climate).action = climate::CLIMATE_ACTION_IDLE;
                  }
                }
                id(my_custom_climate).publish_state();
                break;
              }
              if(datapoint.id == 106) { 
                id(switch_low_protection).publish_state(datapoint.value_bool); 
                break;
              }  
              if(datapoint.id == 107) {
                id(switch_high_protection).publish_state(datapoint.value_bool); 
              }
              break;
            case TuyaDatapointType::INTEGER:
              if (data_size != 4) {
                ESP_LOGW(TAG, "Datapoint %u has bad integer len %zu", datapoint.id, data_size);
                return;
              }
              datapoint.value_int = encode_uint32(data[0], data[1], data[2], data[3]);
              if (!reception.has_value()) {
                if ((datapoint_p->value_int == datapoint.value_int) && datapoint_p.has_value()) {
                  continue;
                } else {
                  if (datapoint.id == reception.value()) {
                    reception.reset();
                  }
                }
              }  
              ESP_LOGD(TAG, "Datapoint %u update to %d", datapoint.id, datapoint.value_int);
              if (datapoint.id == 2) { 
                target_temp_ = datapoint.value_int;
                if(!id(power).state) {
                  id(my_custom_climate).target_temperature = 0.0f;
                } else {   
                  id(my_custom_climate).target_temperature = (float)target_temp_ / 10;  
                }
                id(my_custom_climate).publish_state();
                if (!id(lock).state) {
                  last_blocking_lock = millis();
                }                
                break;
              }  
              if (datapoint.id == 3) {
                id(my_custom_climate).current_temperature = (float)datapoint.value_int / 10; 
                id(my_custom_climate).publish_state();
                break;
              } 
              if (datapoint.id == 103) {
                id(sensor_external).publish_state ((float)datapoint.value_int / 10);
                break;
              }
              reception = datapoint.id;
              if (!id(lock).state) {
                last_blocking_lock = millis();
              } 
              if (datapoint.id == 104) {
                id(number_vacation_days).publish_state (datapoint.value_int);
                break;
              }            
              if (datapoint.id == 105) {
                id(number_vacation_temperature).publish_state(datapoint.value_int);
                break;
              }
              if (datapoint.id == 109) {
                id(number_calibration_sensor).publish_state((float)datapoint.value_int / 10); 
                break;
              }           
              if (datapoint.id == 110) {
                id(number_working_hysteresis).publish_state((float)datapoint.value_int / 10);
                break;
              }           
              if (datapoint.id == 111) { 
                id(number_high_hysteresis_external).publish_state(datapoint.value_int); 
                break;
              }           
              if (datapoint.id == 112) {
                id(number_high_external).publish_state(datapoint.value_int);
                break;
              }           
              if (datapoint.id == 113) {
                id(number_low_external).publish_state(datapoint.value_int); 
                break;
              }
              if (datapoint.id == 114) {
                id(number_max_temperature).publish_state(datapoint.value_int); 
                id(my_custom_climate).set_visual_max_temperature_override(datapoint.value_int);
                break;
              }
              if (datapoint.id == 115) {
                id(number_min_temperature).publish_state(datapoint.value_int);             
                id(my_custom_climate).set_visual_min_temperature_override(datapoint.value_int); 
              }
              break;
            case TuyaDatapointType::ENUM:
              if (data_size != 1) {
                ESP_LOGW(TAG, "Datapoint %u has bad enum len %zu", datapoint.id, data_size);
                return;
              }
              datapoint.value_enum = data[0];
              if (!reception.has_value()) {
                if ((datapoint_p->value_enum == datapoint.value_enum) && datapoint_p.has_value()) {
                continue;
                } else {
                  if (datapoint.id == reception.value()) {
                    reception.reset();
                  }
                }
              }  
              ESP_LOGD(TAG, "Datapoint %u update to %d", datapoint.id, datapoint.value_enum);
              if (!id(lock).state) {
                last_blocking_lock = millis();
              } 
              reception = datapoint.id;
              if (datapoint.id == 4) {
                if (datapoint.value_enum == 0) {id(options_mode).publish_state("Manual");}
                if (datapoint.value_enum == 1) {id(options_mode).publish_state("Program");}
                if (datapoint.value_enum == 2) {id(options_mode).publish_state("Holiday");}
                if (datapoint.value_enum == 3) {id(options_mode).publish_state("TempProg");}
                if (datapoint.value_enum != 3) {
                  if (id(my_global_mode) != datapoint.value_enum) {
                    this->last_press_mode_ = millis();  
                  }
                }
                break;
              }  
              if (datapoint.id == 116) {            
                if (datapoint.value_enum == 0) {id(options_control).publish_state("built-in");}  
                if (datapoint.value_enum == 1) {id(options_control).publish_state("external");}
                if (datapoint.value_enum == 2) {id(options_control).publish_state("built-in_external");} 
                break;
              }      
              if (datapoint.id == 117) {             
                if (datapoint.value_enum == 0) {id(turn_on_state).publish_state("same state");}
                if (datapoint.value_enum == 1) {id(turn_on_state).publish_state("off state");}
                if (datapoint.value_enum == 2) {id(turn_on_state).publish_state("on state");} 
                break;
              }         
              if (datapoint.id == 118) {             
                if (datapoint.value_enum == 0) {id(select_week).publish_state("5+2/2");}
                if (datapoint.value_enum == 1) {id(select_week).publish_state("6+1/1");}
                if (datapoint.value_enum == 2) {id(select_week).publish_state("7 days");}
                break;
              }             
              break;
            case TuyaDatapointType::BITMASK: 
              switch (data_size) { 
                case 1:
                  datapoint.value_bitmask = encode_uint32(0, 0, 0, data[0]);
                  break;
                case 2:
                  datapoint.value_bitmask = encode_uint32(0, 0, data[0], data[1]);
                  break;
                case 4:
                  datapoint.value_bitmask = encode_uint32(data[0], data[1], data[2], data[3]);
                  break;
                default:
                  ESP_LOGW(TAG, "Datapoint %u has bad bitmask len %zu", datapoint.id, data_size);
                  return;
              }
              if ((datapoint_p->value_bitmask == datapoint.value_bitmask) && datapoint_p.has_value()) {continue;} 
              ESP_LOGD(TAG, "Datapoint %u update to %#08X", datapoint.id, datapoint.value_bitmask);
              if (datapoint.id == 12) {
                if (datapoint.value_bitmask == 0) {id(error).publish_state("No");}
                if (datapoint.value_bitmask == 2) {id(error).publish_state("Internal sensor");}
                if (datapoint.value_bitmask == 4) {id(error).publish_state("External sensor");}
                if (datapoint.value_bitmask == 8) {id(error).publish_state("Lower temperature threshold");}
                if (datapoint.value_bitmask == 16) {id(error).publish_state("Upper temperature threshold");} 
              }    
              break;
            default:
              ESP_LOGW(TAG, "Datapoint %u has unknown type %#02hhX", datapoint.id, static_cast<uint8_t>(datapoint.type));
              return;            
          }    

          // Update internal datapoints
          bool found = false;
          for (auto &other : this->datapoints_) {
            if (other.id == datapoint.id) {
              other = datapoint;
              found = true;
            }
          }
          if (!found) {
            this->datapoints_.push_back(datapoint);
          }
          
          if (this->init_state_ == TuyaInitState::INIT_DATAPOINT) {
            if (datapoint.id == 122) {
              this->init_state_ = TuyaInitState::INIT_DONE; 
              this->dump_config();
            }
          }           
        }
      }
   
      optional<TuyaDatapoint> get_datapoint_(uint8_t datapoint_id) {
        for (auto &datapoint : this->datapoints_) {
          if (datapoint.id == datapoint_id)
            return datapoint;
        }
        return {};
      }   

      uint8_t get_wifi_status_code_() {
        uint8_t status = 0x01;
        if (network::is_connected()) {
          status = 0x02;
 
          // Protocol version 1 also supports specifying when connected to "the cloud"
          if (this->protocol_version_ == 0x01 && remote_is_connected()) {
            status = 0x03;
          }  
    
        } else {
     #ifdef USE_CAPTIVE_PORTAL
          if (captive_portal::global_captive_portal != nullptr && captive_portal::global_captive_portal->is_active()) {
            status = 0x00;
        }  
     #endif
        };
        return status;
      }
 
      uint8_t get_wifi_rssi_() {
     #ifdef USE_WIFI
        if (wifi::global_wifi_component != nullptr)
          return wifi::global_wifi_component->wifi_rssi();
     #endif
        return 0;
      }
 
 void send_wifi_status_() {
   uint8_t status = this->get_wifi_status_code_();
    
    if(!remote_is_connected()) {
     uint8_t status_= status;
     }
    
   if (status == this->wifi_status_) {
     return;
   }
  
    if (this->wifi_status_ == 0x03) {
      this->send_command_(TuyaCommand{.cmd = TuyaCommandType::WIFI_STATE, .payload = std::vector<uint8_t>{0x01}});
      this->wifi_status_ = 0x01;
    } else {   
       ESP_LOGD(TAG, "Sending WiFi Status");
       this->wifi_status_ = status;
       this->send_command_(TuyaCommand{.cmd = TuyaCommandType::WIFI_STATE, .payload = std::vector<uint8_t>{status}});
   }
 } 
 
  void set_raw_datapoint_value_(uint8_t datapoint_id, const std::vector<uint8_t> &value, bool forced) {
   ESP_LOGD(TAG, "Setting datapoint %u to %s", datapoint_id, format_hex_pretty(value).c_str());
   optional<TuyaDatapoint> datapoint = this->get_datapoint_(datapoint_id);
   if (!datapoint.has_value()) {
     ESP_LOGW(TAG, "Setting unknown datapoint %u", datapoint_id);
   } else if (datapoint->type != TuyaDatapointType::RAW) {
     ESP_LOGE(TAG, "Attempt to set datapoint %u with incorrect type", datapoint_id);
     return;
   } else if (!forced && datapoint->value_raw == value) {
     ESP_LOGV(TAG, "Not sending unchanged value");
     return;
   }
   this->send_datapoint_command_(datapoint_id, TuyaDatapointType::RAW, value);
 }
 
 void set_numeric_datapoint_value_(uint8_t datapoint_id, TuyaDatapointType datapoint_type, const uint32_t value, uint8_t length, bool forced) {
   ESP_LOGD(TAG, "Setting datapoint %u to %u", datapoint_id, value);
   optional<TuyaDatapoint> datapoint = this->get_datapoint_(datapoint_id);
   if (!datapoint.has_value()) { 
     ESP_LOGW(TAG, "Setting unknown datapoint %u", datapoint_id);
   } else if (datapoint->type != datapoint_type) {
     ESP_LOGE(TAG, "Attempt to set datapoint %u with incorrect type", datapoint_id);
     return;
   } else if (!forced && datapoint->value_uint == value) {
     ESP_LOGV(TAG, "Not sending unchanged value");
     return;
   }
 
   std::vector<uint8_t> data;
   switch (length) {
     case 4:
       data.push_back(value >> 24);
       data.push_back(value >> 16);
     case 2:
       data.push_back(value >> 8);
     case 1:
       data.push_back(value >> 0);
       break;
     default:
       ESP_LOGE(TAG, "Unexpected datapoint length %u", length);
       return;
   }
   this->send_datapoint_command_(datapoint_id, datapoint_type, data);
 } 
 
 void send_datapoint_command_(uint8_t datapoint_id, TuyaDatapointType datapoint_type, std::vector<uint8_t> data) {
   std::vector<uint8_t> buffer;
   buffer.push_back(datapoint_id);
   buffer.push_back(static_cast<uint8_t>(datapoint_type));
   buffer.push_back(data.size() >> 8);
   buffer.push_back(data.size() >> 0);
   buffer.insert(buffer.end(), data.begin(), data.end());
 
   this->send_command_(TuyaCommand{.cmd = TuyaCommandType::DATAPOINT_DELIVER, .payload = buffer});
 } 
 
 
 #ifdef USE_TIME
 void send_local_time_() {
   std::vector<uint8_t> payload;
   auto *time_id = *this->time_id_;
   ESPTime now = time_id->now();
   if (now.is_valid()) {
     uint8_t year = now.year - 2000;
     uint8_t month = now.month;
     uint8_t day_of_month = now.day_of_month;
     uint8_t hour = now.hour;
     uint8_t minute = now.minute;
     uint8_t second = now.second;
     // Tuya days starts from Monday, esphome uses Sunday as day 1
     uint8_t day_of_week = now.day_of_week - 1;
     if (day_of_week == 0) {
       day_of_week = 7;
     }
     ESP_LOGD(TAG, "Sending local time");
     payload = std::vector<uint8_t>{0x01, year, month, day_of_month, hour, minute, second, day_of_week};
   } else {
     // By spec we need to notify MCU that the time was not obtained if this is a rESPonse to a query
     ESP_LOGW(TAG, "Sending missing local time");
     payload = std::vector<uint8_t>{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
   }
   this->send_command_(TuyaCommand{.cmd = TuyaCommandType::LOCAL_TIME_QUERY, .payload = payload});
 }
 #endif 
 
    
};  

class MyCustomClimate : public Component, public Climate {
    public:
   
    void setup() override {

    }
  
    void control(const ClimateCall &call) override {
      if (call.get_target_temperature().has_value()) {
        sending.datapointapi = 2;
        sending.valueapi = *call.get_target_temperature() * 10;
        sending_sensor = sending; 
      }
    }
  
    ClimateTraits traits() override {
      auto traits = climate::ClimateTraits();
      traits.set_visual_temperature_step(0.5);
      traits.set_supported_modes({climate::CLIMATE_MODE_HEAT});
      traits.set_supports_action(true);
      traits.set_supports_current_temperature(true);
      //traits.set_visual_max_temperature((float)id(number_max_temperature).state);
      return traits;
    }
  };
  
  
  class MyCustomSwitch : public Component, public Switch {
    public:
    
      int custom_buttonID;
      MyCustomSwitch(int custom_button) {custom_buttonID = custom_button;}

      void setup() override {

      }       
    
      void write_state(bool state) override {
        
        //power
        if (custom_buttonID == 0) {
          sending.datapointapi = 1;
        } 
        
        //lock
        if (custom_buttonID == 1) {
          sending.datapointapi = 6; 
        } 
        
        //switch_low_protection
        if (custom_buttonID == 2) {
          sending.datapointapi = 106;  
        }     
        
        //switch_low_protection
        if (custom_buttonID == 3) {
          sending.datapointapi = 107;  
        } 
        
        if (custom_buttonID <= 3) {
          sending.valueapi = state;
          sending_switch = sending;
        }
        
        //setting 
        if (custom_buttonID == 4) {
          if (state) {
            if (id(lock).state || !id(power).state) {
              state = !state;
            }  
          } else {
            sending.datapointapi = 5;
            sending.valueapi = state;
            sending_switch = sending;
          }
        } 
        
        
        //setting points
        if (custom_buttonID == 5) {
          if (state) {
            if (!id(setting).state) {
              state = !state;
            }  
          } else {
            sending.datapointapi = 7;
            sending.valueapi = state;
            sending_switch = sending;
          }
        }
        
        //point data (mode programm)to a components number working day
        if (custom_buttonID == 6) {
          if (state) {
            if (id(week_points).state || !id(setting_points).state) {
              state = !state;
            } else {
              sending.datapointapi = 8;
              sending.valueapi = state;
              sending_switch = sending;
            }  
          }    
        }

        //point data (mode programm)to a components number weekend day
        if (custom_buttonID == 7) {
          if (state) {
            if (id(wor_points).state || !id(setting_points).state) {
              state = !state;
            } else {
              sending.datapointapi = 9;
              sending.valueapi = state;
              sending_switch = sending;
            }  
          }    
        }        
        
        //save data point
        if (custom_buttonID == 8) {
          if (state) {
            if (!id(cansel).state) {
              state = !state;
            } else {
              sending_sensor_raw = 119;
            }
          }        
        } 

        //cansel data point
        if (custom_buttonID == 9) {
          if (state) {
            if ((!id(wor_points).state && !id(week_points).state) || (id(wor_points).state && id(week_points).state)) {
              state = !state;
            } 
          }    
        }    
        
        publish_state(state);      

        if (!id(lock).state) {
          last_blocking_lock = millis();
        }        
          
      }
  };
