#include "opentherm_component.h"
#include "esphome/core/log.h"

namespace esphome
{
  namespace opentherm
  {

    static const char *const TAG = "opentherm.component";

    // Initialize static members
    OpenthermComponent *OpenthermComponent::instance_ = nullptr;
    unsigned long OpenthermComponent::last_status_response_ = 0;
    unsigned long OpenthermComponent::last_intercepted_response_ = 0;
    OpenThermMessageID OpenthermComponent::last_intercepted_id_ = static_cast<OpenThermMessageID>(0);
    bool OpenthermComponent::has_new_intercepted_response_ = false;

    OpenthermComponent::OpenthermComponent(uint32_t update_interval) : PollingComponent(update_interval)
    {
      instance_ = this;
    }

    void OpenthermComponent::setup()
    {
      ESP_LOGD(TAG, "Setting up OpenTherm component");

      // Initialize OpenTherm instances
      ot_ = new OpenTherm(in_pin_, out_pin_, false);                  // Master
      slave_ot_ = new OpenTherm(slave_in_pin_, slave_out_pin_, true); // Slave

      // Start OpenTherm communication
      ot_->begin(handleInterrupt);
      slave_ot_->begin(slaveHandleInterrupt, processRequest);

      // Setup climate controllers
      if (hot_water_climate_ != nullptr)
      {
        hot_water_climate_->set_target_temperature_callback([this](float temperature)
                                                            { return this->setHotWaterTemperature(temperature); });
      }

      if (heating_water_climate_ != nullptr)
      {
        heating_water_climate_->set_target_temperature_callback([this](float temperature)
                                                                { return this->setHeatingTargetTemperature(temperature); });
      }

      // Read Phase 1 values once at startup (these don't change)
      delay(1000); // Give OpenTherm time to initialize

      // Read max CH setpoint (Data-ID 57)
      if (max_ch_setpoint_sensor_ != nullptr)
      {
        unsigned long response = ot_->sendRequest(ot_->buildRequest(OpenThermRequestType::READ, OpenThermMessageID::MaxTSet, 0));
        if (ot_->isValidResponse(response))
        {
          float value = ot_->getFloat(response);
          max_ch_setpoint_sensor_->publish_state(value);
          ESP_LOGI(TAG, "Max CH setpoint: %.1f°C", value);
        }
      }

      // Note: Min CH setpoint (Data-ID 58) is not in standard OpenTherm spec
      // Most boilers don't support it, so we skip it

      // Read max relative modulation (Data-ID 14)
      if (max_modulation_sensor_ != nullptr)
      {
        unsigned long response = ot_->sendRequest(ot_->buildRequest(OpenThermRequestType::READ, OpenThermMessageID::MaxRelModLevelSetting, 0));
        if (ot_->isValidResponse(response))
        {
          float value = ot_->getFloat(response);
          max_modulation_sensor_->publish_state(value);
          ESP_LOGI(TAG, "Max modulation: %.1f%%", value);
        }
      }

      // Read OpenTherm versions (Data-ID 124, 125)
      if (master_ot_version_sensor_ != nullptr)
      {
        unsigned long response = ot_->sendRequest(ot_->buildRequest(OpenThermRequestType::READ, OpenThermMessageID::OpenThermVersionMaster, 0));
        if (ot_->isValidResponse(response))
        {
          float value = ot_->getFloat(response);
          master_ot_version_sensor_->publish_state(value);
          ESP_LOGI(TAG, "Master OT version: %.2f", value);
        }
      }

      if (slave_ot_version_sensor_ != nullptr)
      {
        unsigned long response = ot_->sendRequest(ot_->buildRequest(OpenThermRequestType::READ, OpenThermMessageID::OpenThermVersionSlave, 0));
        if (ot_->isValidResponse(response))
        {
          float value = ot_->getFloat(response);
          slave_ot_version_sensor_->publish_state(value);
          ESP_LOGI(TAG, "Slave OT version: %.2f", value);
        }
      }
    }

    void OpenthermComponent::loop()
    {
      slave_ot_->process();

      // Process intercepted responses (moved from interrupt context)
      if (has_new_intercepted_response_)
      {
        processCachedResponse(last_intercepted_response_, last_intercepted_id_);
        has_new_intercepted_response_ = false;
      }
    }

    void OpenthermComponent::update()
    {
      // Read and publish sensor values

      // Binary sensors from status
      bool is_flame_on = ot_->isFlameOn(last_status_response_);
      bool is_central_heating_active = ot_->isCentralHeatingActive(last_status_response_);
      bool is_hot_water_active = ot_->isHotWaterActive(last_status_response_);
      bool is_fault = ot_->isFault(last_status_response_);
      bool is_diagnostic = ot_->isDiagnostic(last_status_response_);

      if (flame_ != nullptr)
        flame_->publish_state(is_flame_on);

      if (ch_active_ != nullptr)
        ch_active_->publish_state(is_central_heating_active);

      if (dhw_active_ != nullptr)
        dhw_active_->publish_state(is_hot_water_active);

      if (fault_ != nullptr)
        fault_->publish_state(is_fault);

      if (diagnostic_ != nullptr)
        diagnostic_->publish_state(is_diagnostic);

      // Temperature and other sensors (using cache with timeout)
      float ext_temperature = getExternalTemperature();
      float return_temperature = getReturnTemperature();
      float boiler_temperature = getCachedOrFetch(cached_boiler_temp_, OpenThermMessageID::Tboiler);
      float pressure = getPressure();
      float modulation = getModulation();
      float heating_target_temp = getHeatingTargetTemperature();
      float hot_water_temp = getHotWaterTemperature();
      float room_temperature = getRoomTemperature();
      float room_setpoint = getRoomSetpoint();

      if (external_temperature_sensor_ != nullptr && !std::isnan(ext_temperature))
        external_temperature_sensor_->publish_state(ext_temperature);

      if (return_temperature_sensor_ != nullptr && !std::isnan(return_temperature))
        return_temperature_sensor_->publish_state(return_temperature);

      if (boiler_temperature_ != nullptr && !std::isnan(boiler_temperature))
        boiler_temperature_->publish_state(boiler_temperature);

      if (pressure_sensor_ != nullptr && !std::isnan(pressure))
        pressure_sensor_->publish_state(pressure);

      if (modulation_sensor_ != nullptr && !std::isnan(modulation))
        modulation_sensor_->publish_state(modulation);

      if (heating_target_temperature_sensor_ != nullptr && !std::isnan(heating_target_temp) && heating_target_temp > 0)
        heating_target_temperature_sensor_->publish_state(heating_target_temp);

      // Room temperature (ID 24) — sent by master (e.g. QAA73) as WRITE-DATA, intercepted from bus
      if (room_temperature_sensor_ != nullptr && !std::isnan(room_temperature))
        room_temperature_sensor_->publish_state(room_temperature);

      // Room setpoint (ID 16) — sent by master (e.g. QAA73) as WRITE-DATA, intercepted from bus
      if (room_setpoint_sensor_ != nullptr && !std::isnan(room_setpoint))
        room_setpoint_sensor_->publish_state(room_setpoint);

      // Read OEM diagnostic codes (Data-ID 5 and 115) - only if fault or diagnostic active
      if (is_fault || is_diagnostic)
      {
        // OEM fault code (Data-ID 5) - Application-specific fault flags
        if (oem_fault_code_sensor_ != nullptr)
        {
          unsigned long response = ot_->sendRequest(ot_->buildRequest(OpenThermRequestType::READ, OpenThermMessageID::ASFflags, 0));
          if (ot_->isValidResponse(response))
          {
            uint16_t fault_code = response & 0xFF; // Low byte contains OEM fault code
            oem_fault_code_sensor_->publish_state(fault_code);
            if (fault_code != 0)
            {
              ESP_LOGW(TAG, "OEM Fault Code: %d", fault_code);
            }
          }
        }

        // OEM diagnostic code (Data-ID 115)
        if (oem_diagnostic_code_sensor_ != nullptr)
        {
          unsigned long response = ot_->sendRequest(ot_->buildRequest(OpenThermRequestType::READ, OpenThermMessageID::OEMDiagnosticCode, 0));
          if (ot_->isValidResponse(response))
          {
            uint16_t diag_code = response & 0xFFFF; // Full 16-bit diagnostic code
            oem_diagnostic_code_sensor_->publish_state(diag_code);
            if (diag_code != 0)
            {
              ESP_LOGW(TAG, "OEM Diagnostic Code: %d", diag_code);
            }
          }
        }
      }
      else
      {
        // No fault - publish 0
        if (oem_fault_code_sensor_ != nullptr)
          oem_fault_code_sensor_->publish_state(0);
        if (oem_diagnostic_code_sensor_ != nullptr)
          oem_diagnostic_code_sensor_->publish_state(0);
      }

      // Update climate controllers
      if (hot_water_climate_ != nullptr)
      {
        hot_water_climate_->current_temperature = hot_water_temp;
        hot_water_climate_->action = is_hot_water_active ? climate::CLIMATE_ACTION_HEATING : climate::CLIMATE_ACTION_OFF;
        
        // Force update DHW target temperature from QAA73 during first 20 update cycles
        // to override any value that HA may have sent during initialization
        static uint8_t dhw_update_counter = 0;
        const uint8_t FORCE_UPDATE_CYCLES = 20;
        
        if (dhw_update_counter < FORCE_UPDATE_CYCLES)
        {
          float dhw_target = getHotWaterTargetTemperature();
          if (!std::isnan(dhw_target) && dhw_target > 0 && dhw_target < 80)
          {
            ESP_LOGI(TAG, "Force updating DHW target to %.1f°C from QAA73 (cycle %d/%d)", 
                     dhw_target, dhw_update_counter + 1, FORCE_UPDATE_CYCLES);
            hot_water_climate_->target_temperature = dhw_target;
          }
          dhw_update_counter++;
        }
        // After force update period, only update if user hasn't overridden it
        else if (!user_dhw_override_active_)
        {
          float dhw_target = getHotWaterTargetTemperature();
          if (!std::isnan(dhw_target) && dhw_target > 0 && dhw_target < 80)
          {
            ESP_LOGV(TAG, "Updating DHW target to %.1f°C from QAA73", dhw_target);
            hot_water_climate_->initialize_target_temperature(dhw_target);
          }
        }
        
        hot_water_climate_->publish_state();
      }

      if (heating_water_climate_ != nullptr)
      {
        // Show room temperature (from master, e.g. QAA73) instead of boiler water temp.
        // Falls back to boiler_temperature if QAA73 hasn't sent Tr yet.
        heating_water_climate_->current_temperature = !std::isnan(room_temperature) ? room_temperature : boiler_temperature;
        heating_water_climate_->action = is_central_heating_active ? climate::CLIMATE_ACTION_HEATING : climate::CLIMATE_ACTION_OFF;
        
        // Only update target temperature if user hasn't overridden it
        if (!user_heating_override_active_)
        {
          // Initialize target from room_setpoint (ID 16, from QAA73) on first update only.
          // Don't use heating_target_temp as fallback - it's CH water temp (40-50°C), not room temp!
          // Only initialize if we have valid room_setpoint from QAA73
          if (!std::isnan(room_setpoint) && room_setpoint > 0)
          {
            heating_water_climate_->initialize_target_temperature(room_setpoint);
          }
        }
        
        heating_water_climate_->publish_state();
      }
    }

    void OpenthermComponent::register_climate(OpenthermClimate *climate)
    {
      ClimateType type = climate->get_climate_type();

      if (type == ClimateType::HOT_WATER)
      {
        hot_water_climate_ = climate;
      }
      else if (type == ClimateType::HEATING_WATER)
      {
        heating_water_climate_ = climate;
      }
    }

    float OpenthermComponent::getExternalTemperature()
    {
      return getCachedOrFetch(cached_external_temp_, OpenThermMessageID::Toutside);
    }

    float OpenthermComponent::getHeatingTargetTemperature()
    {
      return getCachedOrFetch(cached_heating_target_, OpenThermMessageID::TSet);
    }

    float OpenthermComponent::getReturnTemperature()
    {
      return getCachedOrFetch(cached_return_temp_, OpenThermMessageID::Tret);
    }

    float OpenthermComponent::getHotWaterTargetTemperature()
    {
      return getCachedOrFetch(cached_dhw_target_, OpenThermMessageID::TdhwSet);
    }

    float OpenthermComponent::getHotWaterTemperature()
    {
      return getCachedOrFetch(cached_dhw_temp_, OpenThermMessageID::Tdhw);
    }

    float OpenthermComponent::getRoomTemperature()
    {
      // Tr (ID 24) is sent by the master (e.g. QAA73) to the boiler as WRITE-DATA.
      // We intercept it in processRequest() and cache it here.
      // Do NOT use ot_->sendRequest(READ, Tr) — the boiler (slave) does not store
      // this value and will not respond to a READ request for it.
      return cached_room_temp_.value;
    }

    float OpenthermComponent::getRoomSetpoint()
    {
      // TrSet (ID 16) is sent by the master (e.g. QAA73) to the boiler as WRITE-DATA.
      // We intercept it in processRequest() and cache it here.
      // Do NOT use ot_->sendRequest(READ, TrSet) — the boiler (slave) does not store
      // this value and will not respond to a READ request for it.
      return cached_room_setpoint_.value;
    }

    bool OpenthermComponent::setTemperatureWithVerification(
        float temperature,
        OpenThermMessageID write_msg_id,
        OpenThermMessageID read_msg_id,
        OpenthermClimate *climate,
        const char *name)
    {
      ESP_LOGI(TAG, "Setting %s temperature to %.1f°C", name, temperature);

      unsigned int data = ot_->temperatureToData(temperature);
      unsigned long request = ot_->buildRequest(OpenThermRequestType::WRITE, write_msg_id, data);
      unsigned long response = ot_->sendRequest(request);

      if (!ot_->isValidResponse(response))
      {
        ESP_LOGE(TAG, "Failed to set %s temperature - invalid response", name);
        return false;
      }

      // Small delay to allow boiler to process the write command
      delay(100);

      // Verify the setpoint was accepted by reading it back (with retry)
      const int max_retries = 3;
      for (int retry = 0; retry < max_retries; retry++)
      {
        unsigned long read_response = ot_->sendRequest(
            ot_->buildRequest(OpenThermRequestType::READ, read_msg_id, 0));

        if (ot_->isValidResponse(read_response))
        {
          float actual_setpoint = ot_->getFloat(read_response);

          if (!std::isnan(actual_setpoint))
          {
            ESP_LOGI(TAG, "%s setpoint verified: %.1f°C (requested: %.1f°C)",
                     name, actual_setpoint, temperature);

            // Update climate entity immediately with verified value
            if (climate != nullptr)
            {
              climate->target_temperature = actual_setpoint;
              climate->publish_state();
            }

            // Check if setpoint was clamped by boiler (e.g., min/max limits)
            if (std::abs(actual_setpoint - temperature) > 1.0f)
            {
              ESP_LOGW(TAG, "%s setpoint was adjusted by boiler from %.1f°C to %.1f°C (min/max limits?)",
                       name, temperature, actual_setpoint);
            }

            return true;
          }
        }

        // Retry with exponential backoff
        if (retry < max_retries - 1)
        {
          unsigned long backoff = 50 * (1 << retry); // 50ms, 100ms, 200ms
          ESP_LOGW(TAG, "Failed to verify %s setpoint, retry %d/%d after %lu ms",
                   name, retry + 1, max_retries, backoff);
          delay(backoff);
        }
      }

      ESP_LOGW(TAG, "%s setpoint write succeeded but verification failed after %d retries", name, max_retries);
      return true; // Write succeeded even if verification failed
    }

    bool OpenthermComponent::setHotWaterTemperature(float temperature)
    {
      ESP_LOGI(TAG, "User set DHW temperature to %.1f°C", temperature);
      
      // Ignore calls within first 30 seconds after boot - these are from HA restoring state
      unsigned long uptime_ms = millis();
      if (uptime_ms < 30000)
      {
        ESP_LOGI(TAG, "Ignoring DHW temperature set during startup (uptime: %lu ms)", uptime_ms);
        return true;
      }
      
      // Get current QAA73 setpoint
      float qaa73_dhw = getHotWaterTargetTemperature();
      
      // If user sets temperature very close to QAA73's value, don't activate override
      if (!std::isnan(qaa73_dhw) && std::abs(temperature - qaa73_dhw) < 0.5f)
      {
        ESP_LOGI(TAG, "DHW temperature (%.1f°C) matches QAA73 (%.1f°C), not activating override",
                 temperature, qaa73_dhw);
        // Deactivate override if it was active
        user_dhw_override_active_ = false;
        return true;
      }
      
      // Activate user override - this will block QAA73 commands
      user_dhw_override_active_ = true;
      user_dhw_setpoint_ = temperature;
      dhw_override_timestamp_ = millis();
      
      ESP_LOGI(TAG, "DHW override activated: %.1f°C (QAA73 wants %.1f°C)", temperature, qaa73_dhw);
      
      return setTemperatureWithVerification(
          temperature,
          OpenThermMessageID::TdhwSet,
          OpenThermMessageID::TdhwSet,
          hot_water_climate_,
          "DHW");
    }

    bool OpenthermComponent::setHeatingTargetTemperature(float temperature)
    {
      ESP_LOGI(TAG, "User set room temperature to %.1f°C", temperature);
      
      // Ignore calls within first 30 seconds after boot - these are from HA restoring state
      unsigned long uptime_ms = millis();
      if (uptime_ms < 30000)
      {
        ESP_LOGI(TAG, "Ignoring room temperature set during startup (uptime: %lu ms)", uptime_ms);
        return true;
      }
      
      // Get current QAA73 room setpoint
      float qaa73_room_setpoint = getRoomSetpoint();
      
      // If user sets temperature very close to QAA73's value, don't activate override
      if (!std::isnan(qaa73_room_setpoint) && std::abs(temperature - qaa73_room_setpoint) < 0.3f)
      {
        ESP_LOGI(TAG, "Room temperature (%.1f°C) matches QAA73 (%.1f°C), not activating override",
                 temperature, qaa73_room_setpoint);
        // Deactivate override if it was active
        user_heating_override_active_ = false;
        return true;
      }
      
      // Activate user override for room setpoint - this will block QAA73 commands
      user_heating_override_active_ = true;
      user_heating_setpoint_ = temperature;
      heating_override_timestamp_ = millis();
      
      ESP_LOGI(TAG, "Heating override activated: %.1f°C (QAA73 wants %.1f°C)", temperature, qaa73_room_setpoint);
      
      // For heating, we need to set the room setpoint (TrSet/ID 16), not CH water temp (TSet)
      // TrSet is a WRITE-DATA command that tells the boiler what room temperature we want
      ESP_LOGI(TAG, "Setting room setpoint to %.1f°C", temperature);

      unsigned int data = ot_->temperatureToData(temperature);
      unsigned long request = ot_->buildRequest(OpenThermRequestType::WRITE, OpenThermMessageID::TrSet, data);
      unsigned long response = ot_->sendRequest(request);

      if (!ot_->isValidResponse(response))
      {
        ESP_LOGE(TAG, "Failed to set room setpoint - invalid response");
        return false;
      }

      // Update climate entity immediately
      if (heating_water_climate_ != nullptr)
      {
        heating_water_climate_->target_temperature = temperature;
        heating_water_climate_->publish_state();
      }

      ESP_LOGI(TAG, "Room setpoint set to %.1f°C", temperature);
      return true;
    }

    float OpenthermComponent::getModulation()
    {
      return getCachedOrFetch(cached_modulation_, OpenThermMessageID::RelModLevel);
    }

    float OpenthermComponent::getPressure()
    {
      return getCachedOrFetch(cached_pressure_, OpenThermMessageID::CHPressure);
    }

    void OpenthermComponent::processRequest(unsigned long request, OpenThermResponseStatus status)
    {
      if (instance_ != nullptr && instance_->ot_ != nullptr && instance_->slave_ot_ != nullptr)
      {
        OpenThermMessageID id = instance_->ot_->getDataID(request);
        OpenThermMessageType msg_type = instance_->ot_->getMessageType(request);
        
        unsigned long modified_request = request;
        
        // Check if this is a DHW setpoint write from QAA73 and user has overridden it
        if (id == OpenThermMessageID::TdhwSet && 
            msg_type == OpenThermMessageType::WRITE_DATA &&
            instance_->user_dhw_override_active_)
        {
          // Check if override is still active (timeout after 24 hours)
          unsigned long now = millis();
          unsigned long override_age = now - instance_->dhw_override_timestamp_;
          const unsigned long OVERRIDE_TIMEOUT = 24UL * 60UL * 60UL * 1000UL; // 24 hours
          
          if (override_age < OVERRIDE_TIMEOUT)
          {
            // Get QAA73's DHW setpoint
            float qaa73_dhw_temp = instance_->ot_->getFloat(request);
            float user_dhw_temp = instance_->user_dhw_setpoint_;
            
            // Check if user has set the same temperature as QAA73 - if so, disable override
            if (std::abs(qaa73_dhw_temp - user_dhw_temp) < 0.5f)
            {
              instance_->user_dhw_override_active_ = false;
              ESP_LOGI(TAG, "DHW override auto-disabled: User setpoint (%.1f°C) matches QAA73 (%.1f°C)",
                       user_dhw_temp, qaa73_dhw_temp);
              // Don't modify request - let QAA73's value through
            }
            else
            {
              // Replace QAA73's temperature with user's setting
              unsigned int user_data = instance_->ot_->temperatureToData(user_dhw_temp);
              modified_request = instance_->ot_->buildRequest(
                OpenThermRequestType::WRITE,
                OpenThermMessageID::TdhwSet,
                user_data
              );
              
              ESP_LOGI(TAG, "DHW override: QAA73 wants %.1f°C, sending user's %.1f°C instead",
                       qaa73_dhw_temp, user_dhw_temp);
            }
          }
          else
          {
            // Override expired, deactivate it
            instance_->user_dhw_override_active_ = false;
            ESP_LOGI(TAG, "DHW override expired after 24 hours, resuming QAA73 control");
          }
        }
        
        // Check if this is CH water setpoint (TSet) from QAA73 and user has heating override active
        // When user lowers room temp, we need to lower the CH water temperature
        if (id == OpenThermMessageID::TSet && 
            msg_type == OpenThermMessageType::WRITE_DATA &&
            instance_->user_heating_override_active_)
        {
          // Check if override is still active (timeout after 24 hours)
          unsigned long now = millis();
          unsigned long override_age = now - instance_->heating_override_timestamp_;
          const unsigned long OVERRIDE_TIMEOUT = 24UL * 60UL * 60UL * 1000UL; // 24 hours
          
          if (override_age < OVERRIDE_TIMEOUT)
          {
            // Get current room temperature and user's target
            float current_temp = instance_->cached_room_temp_.value;
            float target_temp = instance_->user_heating_setpoint_;
            float qaa73_water_temp = instance_->ot_->getFloat(request);
            float outdoor_temp = instance_->cached_external_temp_.value;
            
            // If current temp is above user's target + hysteresis, force low CH water temp to stop heating
            if (!std::isnan(current_temp) && current_temp > target_temp + 0.2f)
            {
              // Set very low water temperature (20°C) to effectively disable heating
              unsigned int low_temp_data = instance_->ot_->temperatureToData(20.0f);
              modified_request = instance_->ot_->buildRequest(
                OpenThermRequestType::WRITE,
                OpenThermMessageID::TSet,
                low_temp_data
              );
              
              ESP_LOGI(TAG, "Heating override: Lowering CH water temp (QAA73: %.1f°C → 20°C) - room %.1f°C > target %.1f°C",
                       qaa73_water_temp, current_temp, target_temp);
            }
            else if (!std::isnan(current_temp) && current_temp < target_temp - 0.5f)
            {
              // Room is below target - calculate water temp using heating curve
              // Use outdoor temperature to calculate proper water temperature
              float calculated_water_temp = qaa73_water_temp; // Default to QAA73's value
              
              if (!std::isnan(outdoor_temp))
              {
                // Heating curve calculation (typical slope 1.5, can be adjusted)
                // Formula: water_temp = base_temp + curve_slope * (20°C - outdoor_temp)
                // For outdoor -10°C: 30 + 1.5*(20-(-10)) = 30 + 45 = 75°C
                // For outdoor +15°C: 30 + 1.5*(20-15) = 30 + 7.5 = 37.5°C
                const float BASE_TEMP = 25.0f;     // Base water temperature
                const float CURVE_SLOPE = 1.4f;    // Heating curve slope (lower = less aggressive)
                const float DESIGN_ROOM_TEMP = 20.0f; // Design indoor temperature
                
                calculated_water_temp = BASE_TEMP + CURVE_SLOPE * (DESIGN_ROOM_TEMP - outdoor_temp);
                
                // Clamp to reasonable limits
                if (calculated_water_temp < 25.0f) calculated_water_temp = 25.0f;
                if (calculated_water_temp > 75.0f) calculated_water_temp = 75.0f;
                
                ESP_LOGI(TAG, "Heating override: Calculated water temp %.1f°C (outdoor: %.1f°C, QAA73: %.1f°C)",
                         calculated_water_temp, outdoor_temp, qaa73_water_temp);
              }
              else
              {
                ESP_LOGW(TAG, "Heating override: No outdoor temp, using QAA73 calculation (%.1f°C)", qaa73_water_temp);
              }
              
              // Send calculated water temperature
              unsigned int water_temp_data = instance_->ot_->temperatureToData(calculated_water_temp);
              modified_request = instance_->ot_->buildRequest(
                OpenThermRequestType::WRITE,
                OpenThermMessageID::TSet,
                water_temp_data
              );
              
              ESP_LOGI(TAG, "Heating override: Allowing CH (room %.1f°C < target %.1f°C, water temp %.1f°C)",
                       current_temp, target_temp, calculated_water_temp);
            }
            else
            {
              // In hysteresis zone (target-0.5 to target+0.2) - maintain current state
              ESP_LOGV(TAG, "Heating override: Hysteresis zone (room %.1f°C, target %.1f°C)",
                       current_temp, target_temp);
            }
          }
          else
          {
            // Override expired, deactivate it
            instance_->user_heating_override_active_ = false;
            ESP_LOGI(TAG, "Heating override expired after 24 hours, resuming QAA73 control");
          }
        }
        
        // Also override TrSet for consistency (though TSet is what actually controls heating)
        if (id == OpenThermMessageID::TrSet && 
            msg_type == OpenThermMessageType::WRITE_DATA &&
            instance_->user_heating_override_active_)
        {
          // Check if override is still active
          unsigned long now = millis();
          unsigned long override_age = now - instance_->heating_override_timestamp_;
          const unsigned long OVERRIDE_TIMEOUT = 24UL * 60UL * 60UL * 1000UL; // 24 hours
          
          if (override_age < OVERRIDE_TIMEOUT)
          {
            // Get QAA73's room setpoint
            float qaa73_room_setpoint = instance_->ot_->getFloat(request);
            float user_setpoint = instance_->user_heating_setpoint_;
            
            // Check if user has set the same temperature as QAA73 - if so, disable override
            if (std::abs(qaa73_room_setpoint - user_setpoint) < 0.3f)
            {
              instance_->user_heating_override_active_ = false;
              ESP_LOGI(TAG, "Heating override auto-disabled: User setpoint (%.1f°C) matches QAA73 (%.1f°C)",
                       user_setpoint, qaa73_room_setpoint);
              // Don't modify request - let QAA73's value through
            }
            else
            {
              // Replace QAA73's room setpoint with user's setting
              unsigned int user_data = instance_->ot_->temperatureToData(user_setpoint);
              modified_request = instance_->ot_->buildRequest(
                OpenThermRequestType::WRITE,
                OpenThermMessageID::TrSet,
                user_data
              );
              
              ESP_LOGI(TAG, "Heating override: Room setpoint QAA73 %.1f°C → user %.1f°C",
                       qaa73_room_setpoint, user_setpoint);
            }
          }
        }
        
        // Send the (possibly modified) request to boiler
        unsigned long response = instance_->ot_->sendRequest(modified_request);
        instance_->slave_ot_->sendResponse(response);

        // Log intercepted requests at VERBOSE level
        ESP_LOGV(TAG, "Intercepted msg_id %d (type %d), response valid: %s",
                 static_cast<int>(id),
                 static_cast<int>(msg_type),
                 instance_->ot_->isValidResponse(response) ? "yes" : "no");

        // Update status response (critical for binary sensors)
        if (id == OpenThermMessageID::Status)
        {
          last_status_response_ = response;
        }

        // Store response for processing in loop() (outside interrupt context)
        if (instance_->ot_->isValidResponse(response))
        {
          last_intercepted_response_ = response;
          last_intercepted_id_ = id;
          has_new_intercepted_response_ = true;
        }
        // Also cache WRITE-DATA requests (thermostat setting values).
        // This is how we capture Tr (ID 24) and TrSet (ID 16) from the master (e.g. QAA73).
        // The master sends these to the boiler; we sniff them off the bus here.
        else if (msg_type == OpenThermMessageType::WRITE_DATA)
        {
          // For WRITE requests, cache the MODIFIED request if override is active
          if (id == OpenThermMessageID::TdhwSet && instance_->user_dhw_override_active_)
          {
            last_intercepted_response_ = modified_request;
          }
          else if (id == OpenThermMessageID::TrSet && instance_->user_heating_override_active_)
          {
            last_intercepted_response_ = modified_request;
          }
          else
          {
            last_intercepted_response_ = request; // Use original request
          }
          last_intercepted_id_ = id;
          has_new_intercepted_response_ = true;
          ESP_LOGV(TAG, "Caching WRITE-DATA request for msg_id %d", static_cast<int>(id));
        }
      }
    }

    void OpenthermComponent::processCachedResponse(unsigned long response, OpenThermMessageID id)
    {
      // This runs in loop(), not interrupt context - safe to do complex operations
      unsigned long now = millis();

      switch (id)
      {
        case OpenThermMessageID::Toutside:
          cached_external_temp_.value = ot_->getFloat(response);
          cached_external_temp_.last_update = now;
          ESP_LOGV(TAG, "Cached external temp: %.1f°C", cached_external_temp_.value);
          break;

        case OpenThermMessageID::Tret:
          cached_return_temp_.value = ot_->getFloat(response);
          cached_return_temp_.last_update = now;
          ESP_LOGV(TAG, "Cached return temp: %.1f°C", cached_return_temp_.value);
          break;

        case OpenThermMessageID::Tboiler:
          cached_boiler_temp_.value = ot_->getFloat(response);
          cached_boiler_temp_.last_update = now;
          ESP_LOGV(TAG, "Cached boiler temp: %.1f°C", cached_boiler_temp_.value);
          break;

        case OpenThermMessageID::CHPressure:
          cached_pressure_.value = ot_->getFloat(response);
          cached_pressure_.last_update = now;
          ESP_LOGV(TAG, "Cached pressure: %.1f bar", cached_pressure_.value);
          break;

        case OpenThermMessageID::RelModLevel:
          cached_modulation_.value = ot_->getFloat(response);
          cached_modulation_.last_update = now;
          ESP_LOGV(TAG, "Cached modulation: %.1f%%", cached_modulation_.value);
          break;

        case OpenThermMessageID::TSet:
          cached_heating_target_.value = ot_->getFloat(response);
          cached_heating_target_.last_update = now;
          ESP_LOGV(TAG, "Cached heating target: %.1f°C", cached_heating_target_.value);
          break;

        case OpenThermMessageID::Tdhw:
          cached_dhw_temp_.value = ot_->getFloat(response);
          cached_dhw_temp_.last_update = now;
          ESP_LOGV(TAG, "Cached DHW temp: %.1f°C", cached_dhw_temp_.value);
          break;

        case OpenThermMessageID::TdhwSet:
          cached_dhw_target_.value = ot_->getFloat(response);
          cached_dhw_target_.last_update = now;
          ESP_LOGV(TAG, "Cached DHW target: %.1f°C", cached_dhw_target_.value);
          break;

        // Room temperature (ID 24) — WRITE-DATA from master (e.g. QAA73).
        // The master periodically sends the actual room temperature it measures
        // (or receives from a connected room sensor) to the boiler.
        case OpenThermMessageID::Tr:
          cached_room_temp_.value = ot_->getFloat(response);
          cached_room_temp_.last_update = now;
          ESP_LOGV(TAG, "Cached room temp: %.1f°C", cached_room_temp_.value);
          break;

        // Room setpoint (ID 16) — WRITE-DATA from master (e.g. QAA73).
        // The master sends the desired room temperature (as set on the thermostat)
        // to the boiler. This is the target the QAA73 is currently trying to reach.
        case OpenThermMessageID::TrSet:
          cached_room_setpoint_.value = ot_->getFloat(response);
          cached_room_setpoint_.last_update = now;
          ESP_LOGV(TAG, "Cached room setpoint: %.1f°C", cached_room_setpoint_.value);
          break;

        case OpenThermMessageID::Status:
          // Already handled in processRequest for immediate binary sensor updates
          ESP_LOGD(TAG, "Updated status response: %lu", response);
          break;

        default:
          // Other message IDs not cached
          break;
      }
    }

    float OpenthermComponent::getCachedOrFetch(CachedValue &cache, OpenThermMessageID msg_id)
    {
      unsigned long now = millis();

      // Handle first fetch (cache never updated) - last_update will be 0
      if (cache.last_update == 0)
      {
        ESP_LOGV(TAG, "First fetch for msg_id %d", static_cast<int>(msg_id));
        unsigned long response = ot_->sendRequest(ot_->buildRequest(OpenThermRequestType::READ, msg_id, 0));

        if (ot_->isValidResponse(response))
        {
          cache.value = ot_->getFloat(response);
          cache.last_update = now;
          ESP_LOGV(TAG, "First fetch for msg_id %d: %.2f", static_cast<int>(msg_id), cache.value);
          return cache.value;
        }
        else
        {
          ESP_LOGW(TAG, "First fetch failed for msg_id %d", static_cast<int>(msg_id));
          cache.last_update = now; // Set timestamp to prevent immediate retry
          return NAN;
        }
      }

      // Unsigned arithmetic handles millis() overflow correctly (wraps at 2^32)
      unsigned long cache_age = now - cache.last_update;

      // Check if cache is fresh (updated within last minute)
      if (!std::isnan(cache.value) && cache_age < CACHE_TIMEOUT_)
      {
        ESP_LOGV(TAG, "Using cached value for msg_id %d: %.2f (age: %lu ms)",
                 static_cast<int>(msg_id), cache.value, cache_age);
        return cache.value;
      }

      // Rate limiting: Don't fetch if we just fetched recently (prevents spam if cache keeps expiring)
      if (cache_age < MIN_FETCH_INTERVAL_)
      {
        ESP_LOGV(TAG, "Rate limited fetch for msg_id %d (last fetch %lu ms ago, min interval %lu ms)",
                 static_cast<int>(msg_id), cache_age, MIN_FETCH_INTERVAL_);
        return cache.value; // Return stale value rather than spam the bus
      }

      // Cache is stale - fetch from boiler
      ESP_LOGV(TAG, "Cache stale for msg_id %d (age: %lu ms), fetching from boiler",
               static_cast<int>(msg_id), cache_age);
      unsigned long response = ot_->sendRequest(ot_->buildRequest(OpenThermRequestType::READ, msg_id, 0));

      if (ot_->isValidResponse(response))
      {
        cache.value = ot_->getFloat(response);
        cache.last_update = now;
        ESP_LOGV(TAG, "Fetched value for msg_id %d: %.2f", static_cast<int>(msg_id), cache.value);
        return cache.value;
      }
      else
      {
        ESP_LOGW(TAG, "Failed to fetch value for msg_id %d, using stale cache if available", static_cast<int>(msg_id));
        // Update timestamp even on failure to prevent continuous retry spam
        cache.last_update = now;
      }

      return cache.value; // Return stale value or NAN
    }

    void IRAM_ATTR OpenthermComponent::handleInterrupt()
    {
      if (instance_ != nullptr && instance_->ot_ != nullptr)
      {
        instance_->ot_->handleInterrupt();
      }
    }

    void IRAM_ATTR OpenthermComponent::slaveHandleInterrupt()
    {
      if (instance_ != nullptr && instance_->slave_ot_ != nullptr)
      {
        instance_->slave_ot_->handleInterrupt();
      }
    }

    bool OpenthermComponent::sendBoilerReset()
    {
      ESP_LOGW(TAG, "Sending Boiler Lock-Out Reset (BLOR) command");

      // Build WRITE-DATA command with Command-Code 1 (BLOR) as per OpenTherm spec section 5.3.3
      unsigned long request = ot_->buildRequest(
          OpenThermRequestType::WRITE,
          OpenThermMessageID::Command,  // Data ID 4
          0x0100);                      // HB=1 (BLOR command), LB=0

      ESP_LOGD(TAG, "BLOR request: 0x%08lX", request);
      unsigned long response = ot_->sendRequest(request);
      ESP_LOGD(TAG, "BLOR response: 0x%08lX", response);

      if (ot_->isValidResponse(response))
      {
        // Extract full response data
        uint16_t response_data = response & 0xFFFF;
        uint8_t high_byte = (response_data >> 8) & 0xFF;
        uint8_t low_byte = response_data & 0xFF;

        ESP_LOGD(TAG, "BLOR response data: HB=0x%02X (%d), LB=0x%02X (%d)",
                 high_byte, high_byte, low_byte, low_byte);

        // Check if command was accepted (response code in LB should be >= 128 for success)
        // Or check HB for echo of command code
        if (low_byte >= 128 || high_byte == 1)
        {
          ESP_LOGI(TAG, "Boiler reset command completed successfully (HB=%d, LB=%d)", high_byte, low_byte);
          return true;
        }
        else
        {
          ESP_LOGW(TAG, "Boiler reset command failed or not supported (HB=%d, LB=%d)", high_byte, low_byte);
          return false;
        }
      }

      ESP_LOGE(TAG, "Boiler reset command - no valid response");
      return false;
    }

  } // namespace opentherm
} // namespace esphome
