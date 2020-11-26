// Auto-generated. Do not edit!

// (in-package logger_service.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class ToggleLoggingRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.loggingEnabled = null;
    }
    else {
      if (initObj.hasOwnProperty('loggingEnabled')) {
        this.loggingEnabled = initObj.loggingEnabled
      }
      else {
        this.loggingEnabled = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ToggleLoggingRequest
    // Serialize message field [loggingEnabled]
    bufferOffset = _serializer.bool(obj.loggingEnabled, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ToggleLoggingRequest
    let len;
    let data = new ToggleLoggingRequest(null);
    // Deserialize message field [loggingEnabled]
    data.loggingEnabled = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'logger_service/ToggleLoggingRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a0bf3b89fe2a5cd89cc4d28dc3bf08ee';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool loggingEnabled
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ToggleLoggingRequest(null);
    if (msg.loggingEnabled !== undefined) {
      resolved.loggingEnabled = msg.loggingEnabled;
    }
    else {
      resolved.loggingEnabled = false
    }

    return resolved;
    }
};

class ToggleLoggingResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.loggingStatus = null;
    }
    else {
      if (initObj.hasOwnProperty('loggingStatus')) {
        this.loggingStatus = initObj.loggingStatus
      }
      else {
        this.loggingStatus = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ToggleLoggingResponse
    // Serialize message field [loggingStatus]
    bufferOffset = _serializer.bool(obj.loggingStatus, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ToggleLoggingResponse
    let len;
    let data = new ToggleLoggingResponse(null);
    // Deserialize message field [loggingStatus]
    data.loggingStatus = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'logger_service/ToggleLoggingResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5a9e683ce2bfe939395c449b32525bad';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool loggingStatus
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ToggleLoggingResponse(null);
    if (msg.loggingStatus !== undefined) {
      resolved.loggingStatus = msg.loggingStatus;
    }
    else {
      resolved.loggingStatus = false
    }

    return resolved;
    }
};

module.exports = {
  Request: ToggleLoggingRequest,
  Response: ToggleLoggingResponse,
  md5sum() { return 'fecf66bb22f64ee8735af9d41647765f'; },
  datatype() { return 'logger_service/ToggleLogging'; }
};
