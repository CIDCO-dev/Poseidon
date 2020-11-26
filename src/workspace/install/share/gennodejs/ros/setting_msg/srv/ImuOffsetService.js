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

class ImuOffsetServiceRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.headingOffset = null;
      this.pitchOffset = null;
      this.rollOffset = null;
    }
    else {
      if (initObj.hasOwnProperty('headingOffset')) {
        this.headingOffset = initObj.headingOffset
      }
      else {
        this.headingOffset = 0.0;
      }
      if (initObj.hasOwnProperty('pitchOffset')) {
        this.pitchOffset = initObj.pitchOffset
      }
      else {
        this.pitchOffset = 0.0;
      }
      if (initObj.hasOwnProperty('rollOffset')) {
        this.rollOffset = initObj.rollOffset
      }
      else {
        this.rollOffset = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ImuOffsetServiceRequest
    // Serialize message field [headingOffset]
    bufferOffset = _serializer.float64(obj.headingOffset, buffer, bufferOffset);
    // Serialize message field [pitchOffset]
    bufferOffset = _serializer.float64(obj.pitchOffset, buffer, bufferOffset);
    // Serialize message field [rollOffset]
    bufferOffset = _serializer.float64(obj.rollOffset, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ImuOffsetServiceRequest
    let len;
    let data = new ImuOffsetServiceRequest(null);
    // Deserialize message field [headingOffset]
    data.headingOffset = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [pitchOffset]
    data.pitchOffset = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [rollOffset]
    data.rollOffset = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 24;
  }

  static datatype() {
    // Returns string type for a service object
    return 'setting_msg/ImuOffsetServiceRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '13af769267c513b8010dc26e66cdc686';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 headingOffset
    float64 pitchOffset
    float64 rollOffset
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ImuOffsetServiceRequest(null);
    if (msg.headingOffset !== undefined) {
      resolved.headingOffset = msg.headingOffset;
    }
    else {
      resolved.headingOffset = 0.0
    }

    if (msg.pitchOffset !== undefined) {
      resolved.pitchOffset = msg.pitchOffset;
    }
    else {
      resolved.pitchOffset = 0.0
    }

    if (msg.rollOffset !== undefined) {
      resolved.rollOffset = msg.rollOffset;
    }
    else {
      resolved.rollOffset = 0.0
    }

    return resolved;
    }
};

class ImuOffsetServiceResponse {
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
    // Serializes a message object of type ImuOffsetServiceResponse
    // Serialize message field [value]
    bufferOffset = _serializer.string(obj.value, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ImuOffsetServiceResponse
    let len;
    let data = new ImuOffsetServiceResponse(null);
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
    return 'setting_msg/ImuOffsetServiceResponse';
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
    const resolved = new ImuOffsetServiceResponse(null);
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
  Request: ImuOffsetServiceRequest,
  Response: ImuOffsetServiceResponse,
  md5sum() { return 'f17e28b74ba2da00db559b27e3cb5a7f'; },
  datatype() { return 'setting_msg/ImuOffsetService'; }
};
