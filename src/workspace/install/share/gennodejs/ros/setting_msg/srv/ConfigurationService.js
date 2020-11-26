// Auto-generated. Do not edit!

// (in-package setting_msg.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class ConfigurationServiceRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.key = null;
    }
    else {
      if (initObj.hasOwnProperty('key')) {
        this.key = initObj.key
      }
      else {
        this.key = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ConfigurationServiceRequest
    // Serialize message field [key]
    bufferOffset = _serializer.string(obj.key, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ConfigurationServiceRequest
    let len;
    let data = new ConfigurationServiceRequest(null);
    // Deserialize message field [key]
    data.key = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.key.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'setting_msg/ConfigurationServiceRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e0e6b82860d1fefbdaa2c52c0b6f712e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string key
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ConfigurationServiceRequest(null);
    if (msg.key !== undefined) {
      resolved.key = msg.key;
    }
    else {
      resolved.key = ''
    }

    return resolved;
    }
};

class ConfigurationServiceResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.value = null;
    }
    else {
      if (initObj.hasOwnProperty('value')) {
        this.value = initObj.value
      }
      else {
        this.value = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ConfigurationServiceResponse
    // Serialize message field [value]
    bufferOffset = _serializer.string(obj.value, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ConfigurationServiceResponse
    let len;
    let data = new ConfigurationServiceResponse(null);
    // Deserialize message field [value]
    data.value = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.value.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'setting_msg/ConfigurationServiceResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '64e58419496c7248b4ef25731f88b8c3';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string value
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ConfigurationServiceResponse(null);
    if (msg.value !== undefined) {
      resolved.value = msg.value;
    }
    else {
      resolved.value = ''
    }

    return resolved;
    }
};

module.exports = {
  Request: ConfigurationServiceRequest,
  Response: ConfigurationServiceResponse,
  md5sum() { return 'c7428de72b961d5dc25117ffa14882f8'; },
  datatype() { return 'setting_msg/ConfigurationService'; }
};
