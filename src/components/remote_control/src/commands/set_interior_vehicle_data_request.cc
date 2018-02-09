/*
 Copyright (c) 2017, Ford Motor Company
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

 Redistributions of source code must retain the above copyright notice, this
 list of conditions and the following disclaimer.

 Redistributions in binary form must reproduce the above copyright notice,
 this list of conditions and the following
 disclaimer in the documentation and/or other materials provided with the
 distribution.

 Neither the name of the Ford Motor Company nor the names of its contributors
 may be used to endorse or promote products derived from this software
 without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 POSSIBILITY OF SUCH DAMAGE.
 */

#include "remote_control/commands/set_interior_vehicle_data_request.h"
#include <algorithm>
#include "remote_control/rc_module_constants.h"
#include "remote_control/message_helper.h"
#include "functional_module/function_ids.h"
#include "json/json.h"
#include "utils/helpers.h"

namespace remote_control {

namespace commands {

using namespace json_keys;
using namespace message_params;

namespace {
std::vector<std::string> GetModuleReadOnlyParams(
    const std::string& module_type) {
  std::vector<std::string> module_ro_params;
  if (enums_value::kClimate == module_type) {
    module_ro_params.push_back(kCurrentTemperature);
  } else if (enums_value::kRadio == module_type) {
    module_ro_params.push_back(kRdsData);
    module_ro_params.push_back(kAvailableHDs);
    module_ro_params.push_back(kSignalStrength);
    module_ro_params.push_back(kSignalChangeThreshold);
    module_ro_params.push_back(kState);
  }
  return module_ro_params;
}

const std::map<std::string, std::string> GetModuleDataToCapabilitiesMapping() {
  std::map<std::string, std::string> mapping;
  // climate
  mapping["fanSpeed"] = "fanSpeedAvailable";
  mapping["currentTemperature"] = "currentTemperatureAvailable";
  mapping["desiredTemperature"] = "desiredTemperatureAvailable";
  mapping["acEnable"] = "acEnableAvailable";
  mapping["circulateAirEnable"] = "circulateAirEnableAvailable";
  mapping["autoModeEnable"] = "autoModeEnableAvailable";
  mapping["defrostZone"] = "defrostZoneAvailable";
  mapping["dualModeEnable"] = "dualModeEnableAvailable";
  mapping["acMaxEnable"] = "acMaxEnableAvailable";
  mapping["ventilationMode"] = "ventilationModeAvailable";
  mapping["heatedSteeringWheelEnable"] = "heatedSteeringWheelAvailable";
  mapping["heatedWindshieldEnable"] = "heatedWindshieldAvailable";
  mapping["heatedRearWindowEnable"] = "heatedRearWindowAvailable";
  mapping["heatedMirrorsEnable"] = "heatedMirrorsAvailable";

  // radio
  mapping["band"] = "radioBandAvailable";
  mapping["frequencyInteger"] = "radioFrequencyAvailable";
  mapping["frequencyFraction"] = "radioFrequencyAvailable";
  mapping["rdsData"] = "rdsDataAvailable";
  mapping["availableHDs"] = "availableHDsAvailable";
  mapping["hdChannel"] = "hdChannelAvailable";
  mapping["signalStrength"] = "signalStrengthAvailable";
  mapping["signalChangeThreshold"] = "signalChangeThresholdAvailable";
  mapping["radioEnable"] = "radioEnableAvailable";
  mapping["state"] = "stateAvailable";
  mapping["sisData"] = "sisDataAvailable";

  //  seat
  mapping["heatingEnabled"] = "heatingEnabledAvailable";
  mapping["coolingEnabled"] = "coolingEnabledAvailable";
  mapping["heatingLevel"] = "heatingLevelAvailable";
  mapping["coolingLevel"] = "coolingLevelAvailable";
  mapping["horizontalPosition"] = "horizontalPositionAvailable";
  mapping["verticalPosition"] = "verticalPositionAvailable";
  mapping["frontVerticalPosition"] = "frontVerticalPositionAvailable";
  mapping["backVerticalPosition"] = "backVerticalPositionAvailable";
  mapping["backTiltAngle"] = "backTiltAngleAvailable";
  mapping["headSupportHorizontalPosition"] = "headSupportHorizontalPositionAvailable";
  mapping["headSupportVerticalPosition"] = "headSupportVerticalPositionAvailable";
  mapping["massageEnabled"] = "massageEnabledAvailable";
  mapping["massageMode"] = "massageModeAvailable";
  mapping["massageCushionFirmness"] = "massageCushionFirmnessAvailable";
  mapping["memory"] = "memoryAvailable";

  //  Audio
  mapping["source"] = "sourceAvailable";
  mapping["volume"] = "volumeAvailable";
  mapping["equalizerSettings"] = "equalizerAvailable";

  //  Hmi Setting
  mapping["displayMode"] = "displayModeUnitAvailable";
  mapping["temperatureUnit"] = "temperatureUnitAvailable";
  mapping["distanceUnit"] = "distanceUnitAvailable";

  return mapping;
}
}  // namespace

CREATE_LOGGERPTR_GLOBAL(logger_, "SetInteriorVehicleDataRequest")

SetInteriorVehicleDataRequest::SetInteriorVehicleDataRequest(
    const application_manager::MessagePtr& message,
    RemotePluginInterface& rc_module)
    : BaseCommandRequest(message, rc_module) {}

SetInteriorVehicleDataRequest::~SetInteriorVehicleDataRequest() {}

bool CheckControlDataByCapabilities(
    const smart_objects::SmartObject& module_caps,
    const Json::Value& control_data) {
  std::map<std::string, std::string> mapping =
      GetModuleDataToCapabilitiesMapping();
  Json::Value::Members control_data_keys = control_data.getMemberNames();

  const smart_objects::SmartObject& capabilities_status = ((smart_objects::SmartType::SmartType_Array == module_caps.getType()) ? module_caps[0] : module_caps);

  Json::Value::Members::const_iterator it = control_data_keys.begin();
  for (; it != control_data_keys.end(); ++it) {
    const std::string& request_parameter = *it;     
    const std::string& caps_key = mapping[request_parameter];

    if(caps_key.empty()){
        continue;
    }

    LOG4CXX_DEBUG(logger_,
                  "Checking request parameter "
                      << request_parameter
                      << " with capabilities. Appropriate key is " << caps_key);
    if (!capabilities_status.keyExists(caps_key)) {
      LOG4CXX_DEBUG(logger_,
                    "Capability "
                        << caps_key
                        << " is missed in RemoteControl capabilities");
      return false;
    }
    if (!capabilities_status[caps_key].asBool()) {
      LOG4CXX_DEBUG(logger_,
                    "Capability "
                        << caps_key
                        << " is switched off in RemoteControl capabilities");
      return false;
    }
  }
  return true;
}

bool CheckIfModuleDataExistInCapabilities(
    const smart_objects::SmartObject& rc_capabilities,
    const Json::Value& module_data) {
  LOG4CXX_AUTO_TRACE(logger_);
  bool is_radio_data_valid = true;
  bool is_climate_data_valid = true;
  bool is_seat_data_valid = true;
  bool is_audio_data_valid = true;
  bool is_light_data_valid = true;
  bool is_hmi_setting_data_valid = true;
  if (IsMember(module_data, kRadioControlData)) {
    if (!rc_capabilities.keyExists(strings::kradioControlCapabilities)) {
      LOG4CXX_DEBUG(logger_, " Radio control capabilities not present");
      return false;
    }
    const smart_objects::SmartObject& radio_caps =
        rc_capabilities[strings::kradioControlCapabilities];
    is_radio_data_valid = CheckControlDataByCapabilities(
        radio_caps, module_data[strings::kRadioControlData]);
  }
  if (IsMember(module_data, kClimateControlData)) {
    if (!rc_capabilities.keyExists(strings::kclimateControlCapabilities)) {
      LOG4CXX_DEBUG(logger_, " Climate control capabilities not present");
      return false;
    }
    const smart_objects::SmartObject& climate_caps =
        rc_capabilities[strings::kclimateControlCapabilities];
    is_climate_data_valid = CheckControlDataByCapabilities(
        climate_caps, module_data[strings::kClimateControlData]);
  }
  if (IsMember(module_data, kSeatControlData)) {
    if (!rc_capabilities.keyExists(strings::kseatControlCapabilities)) {
      LOG4CXX_DEBUG(logger_, " Seat control capabilities not present");
      return false;
    }
    const smart_objects::SmartObject& seat_caps =
        rc_capabilities[strings::kseatControlCapabilities];
    is_seat_data_valid = CheckControlDataByCapabilities(
        seat_caps, module_data[strings::kSeatControlData]);
  }
  if (IsMember(module_data, kAudioControlData)) {
    if (!rc_capabilities.keyExists(strings::kaudioControlCapabilities)) {
      LOG4CXX_DEBUG(logger_, " Audio control capabilities not present");
      return false;
    }
    const smart_objects::SmartObject& audio_caps =
        rc_capabilities[strings::kaudioControlCapabilities];
    is_audio_data_valid = CheckControlDataByCapabilities(
        audio_caps, module_data[strings::kAudioControlData]);
  }
  if (IsMember(module_data, kLightControlData)) {
    if (!rc_capabilities.keyExists(strings::klightControlCapabilities)) {
      LOG4CXX_DEBUG(logger_, " Light control capabilities not present");
      return false;
    }
    const smart_objects::SmartObject& light_caps =
        rc_capabilities[strings::klightControlCapabilities];
    is_light_data_valid = CheckControlDataByCapabilities(
        light_caps, module_data[strings::kLightControlData]);
  }
  if (IsMember(module_data, kHMISettingsControlData)) {
    if (!rc_capabilities.keyExists(strings::khmiSettingsControlCapabilities)) {
      LOG4CXX_DEBUG(logger_, " Hmi Setting control capabilities not present");
      return false;
    }
    const smart_objects::SmartObject& hmiSetting_caps =
        rc_capabilities[strings::khmiSettingsControlCapabilities];
    is_hmi_setting_data_valid = CheckControlDataByCapabilities(
        hmiSetting_caps, module_data[strings::kHMISettingsControlData]);
  }

  return is_radio_data_valid && is_climate_data_valid && is_seat_data_valid && is_audio_data_valid && is_light_data_valid && is_hmi_setting_data_valid;
}

void SetInteriorVehicleDataRequest::Execute() {
  LOG4CXX_AUTO_TRACE(logger_);

  Json::Value request_params =
      MessageHelper::StringToValue(message_->json_message());
  const Json::Value module_data = request_params[kModuleData];
  const std::string module_type = module_data[kModuleType].asString();
  bool module_type_and_data_match = true;

  if (enums_value::kRadio == module_type) {
    module_type_and_data_match = !IsMember(module_data, kClimateControlData);
  }

  if (enums_value::kClimate == module_type) {
    module_type_and_data_match = !IsMember(module_data, kRadioControlData);
  }

  if (module_type_and_data_match) {
    const smart_objects::SmartObject* capabilities =
        service()->GetRCCapabilities();

    if (capabilities &&
            !CheckIfModuleDataExistInCapabilities(*capabilities, module_data)) {
      LOG4CXX_WARN(logger_, "Accessing not supported module data");
      SendResponse(false,
                   result_codes::kUnsupportedResource,
                   "Accessing not supported module data");
      return;
    }
    if (AreAllParamsReadOnly(request_params)) {
      LOG4CXX_WARN(logger_, "All request params in module type are READ ONLY!");
      SendResponse(false,
                   result_codes::kReadOnly,
                   "All request params in module type are READ ONLY!");
      return;
    }
    if (AreReadOnlyParamsPresent(request_params)) {
      LOG4CXX_DEBUG(logger_, "Request module type has READ ONLY parameters");
      LOG4CXX_DEBUG(logger_, "Cutting-off READ ONLY parameters... ");
      CutOffReadOnlyParams(request_params);
    }
    application_manager::MessagePtr hmi_request = CreateHmiRequest(
        functional_modules::hmi_api::set_interior_vehicle_data, request_params);
    service()->RemoveHMIFakeParameters(hmi_request);
    SendMessageToHMI(hmi_request);
  } else {
    LOG4CXX_WARN(logger_, "Request module type & data mismatch!");
    SendResponse(false,
                 result_codes::kInvalidData,
                 "Request module type & data mismatch!");
  }
}

AcquireResult::eType SetInteriorVehicleDataRequest::AcquireResource(
    const Json::Value& message) {
  return rc_module_.resource_allocation_manager().AcquireResource(
      ModuleType(message), app()->app_id());
}

bool SetInteriorVehicleDataRequest::IsResourceFree(
    const std::string& module_type) const {
  return rc_module_.resource_allocation_manager().IsResourceFree(module_type);
}

void SetInteriorVehicleDataRequest::SetResourceState(
    const Json::Value& message, const ResourceState::eType state) {
  const std::string& module_type = ModuleType(message);
  const uint32_t app_id = app()->app_id();

  ResourceAllocationManager& allocation_manager =
      rc_module_.resource_allocation_manager();

  allocation_manager.SetResourceState(module_type, app_id, state);
}

bool SetInteriorVehicleDataRequest::AreReadOnlyParamsPresent(
    const Json::Value& request_params) {
  LOG4CXX_AUTO_TRACE(logger_);
  std::vector<std::string> module_type_params = ControlData(request_params);
  std::vector<std::string>::iterator it = module_type_params.begin();
  std::vector<std::string> ro_params =
      GetModuleReadOnlyParams(ModuleType(request_params));
  for (; it != module_type_params.end(); ++it) {
    if (helpers::in_range(ro_params, *it)) {
      return true;
    }
  }
  return false;
}

void SetInteriorVehicleDataRequest::CutOffReadOnlyParams(
    Json::Value& request_params) {
  LOG4CXX_AUTO_TRACE(logger_);
  std::vector<std::string> module_type_params = ControlData(request_params);
  std::vector<std::string>::iterator it = module_type_params.begin();
  const std::string module_type = ModuleType(request_params);
  std::vector<std::string> ro_params = GetModuleReadOnlyParams(module_type);
  for (; it != module_type_params.end(); ++it) {
    if (helpers::in_range(ro_params, *it)) {
      if (enums_value::kClimate == module_type) {
        request_params[message_params::kModuleData]
                      [message_params::kClimateControlData].removeMember(*it);
        LOG4CXX_DEBUG(logger_, "Cutting-off READ ONLY parameter: " << *it);
      } else if (enums_value::kRadio == module_type) {
        request_params[message_params::kModuleData]
                      [message_params::kRadioControlData].removeMember(*it);
        LOG4CXX_DEBUG(logger_, "Cutting-off READ ONLY parameter: " << *it);
      } else if (enums_value::kSeat == module_type) {
          request_params[message_params::kModuleData]
                        [message_params::kSeatControlData].removeMember(*it);
          LOG4CXX_DEBUG(logger_, "Cutting-off READ ONLY parameter: " << *it);
      }
    }
  }
}

bool SetInteriorVehicleDataRequest::AreAllParamsReadOnly(
    const Json::Value& request_params) {
  LOG4CXX_AUTO_TRACE(logger_);
  std::vector<std::string> module_type_params = ControlData(request_params);
  std::vector<std::string>::iterator it = module_type_params.begin();
  std::vector<std::string> ro_params =
      GetModuleReadOnlyParams(ModuleType(request_params));
  for (; it != module_type_params.end(); ++it) {
    if (!helpers::in_range(ro_params, *it)) {
      return false;
    }
  }
  return true;
}

void SetInteriorVehicleDataRequest::OnEvent(
    const rc_event_engine::Event<application_manager::MessagePtr, std::string>&
        event) {
  LOG4CXX_AUTO_TRACE(logger_);
  DCHECK_OR_RETURN_VOID(
      (functional_modules::hmi_api::set_interior_vehicle_data == event.id()));

  application_manager::Message& hmi_response = *(event.event_message());
  if (application_manager::MessageValidationResult::SUCCESS !=
      service()->ValidateMessageBySchema(hmi_response)) {
    SendResponse(false, result_codes::kGenericError, "");
    return;
  }

  LOG4CXX_DEBUG(logger_, "HMI response is valid");
  const Json::Value value =
      MessageHelper::StringToValue(hmi_response.json_message());

  std::string result_code;
  std::string info;

  const bool is_response_successful = ParseResultCode(value, result_code, info);

  if (is_response_successful) {
    response_params_[message_params::kModuleData] =
        value[json_keys::kResult][message_params::kModuleData];
  }

  SendResponse(is_response_successful, result_code.c_str(), info);
}

std::string SetInteriorVehicleDataRequest::ModuleType(
    const Json::Value& message) {
  const Json::Value& module_data =
      message.get(message_params::kModuleData, Json::Value(Json::objectValue));
  return module_data.get(message_params::kModuleType, "").asString();
}

std::vector<std::string> SetInteriorVehicleDataRequest::ControlData(
    const Json::Value& message) {
  Json::Value data =
      message.get(message_params::kModuleData, Json::Value(Json::objectValue));
  std::string module = ModuleType(message);
  Json::Value params;
  if (module == enums_value::kRadio) {
    params = data.get(message_params::kRadioControlData,
                      Json::Value(Json::objectValue));
  }
  if (module == enums_value::kClimate) {
    params = data.get(message_params::kClimateControlData,
                      Json::Value(Json::objectValue));
  }

  if (module == enums_value::kSeat) {
    params = data.get(message_params::kSeatControlData,
                      Json::Value(Json::objectValue));
  }

  if (module == enums_value::kAudio) {
    params = data.get(message_params::kAudioControlData,
                      Json::Value(Json::objectValue));
  }

  if (module == enums_value::kLight) {
    params = data.get(message_params::kLightControlData,
                      Json::Value(Json::objectValue));
  }

  if (module == enums_value::kHmiSettings) {
    params = data.get(message_params::kHMISettingsControlData,
                      Json::Value(Json::objectValue));
  }

  return params.getMemberNames();
}

}  // namespace commands

}  // namespace remote_control
