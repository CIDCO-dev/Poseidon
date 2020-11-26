// Auto-generated. Do not edit!

// (in-package raspberrypi_vitals_msg.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class sysinfo {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.cputemp = null;
      this.cpuload = null;
      this.freeram = null;
      this.freehdd = null;
      this.uptime = null;
      this.vbat = null;
      this.rh = null;
      this.temp = null;
      this.psi = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = 0.0;
      }
      if (initObj.hasOwnProperty('cputemp')) {
        this.cputemp = initObj.cputemp
      }
      else {
        this.cputemp = 0.0;
      }
      if (initObj.hasOwnProperty('cpuload')) {
        this.cpuload = initObj.cpuload
      }
      else {
        this.cpuload = 0.0;
      }
      if (initObj.hasOwnProperty('freeram')) {
        this.freeram = initObj.freeram
      }
      else {
        this.freeram = 0.0;
      }
      if (initObj.hasOwnProperty('freehdd')) {
        this.freehdd = initObj.freehdd
      }
      else {
        this.freehdd = 0.0;
      }
      if (initObj.hasOwnProperty('uptime')) {
        this.uptime = initObj.uptime
      }
      else {
        this.uptime = 0.0;
      }
      if (initObj.hasOwnProperty('vbat')) {
        this.vbat = initObj.vbat
      }
      else {
        this.vbat = 0.0;
      }
      if (initObj.hasOwnProperty('rh')) {
        this.rh = initObj.rh
      }
      else {
        this.rh = 0.0;
      }
      if (initObj.hasOwnProperty('temp')) {
        this.temp = initObj.temp
      }
      else {
        this.temp = 0.0;
      }
      if (initObj.hasOwnProperty('psi')) {
        this.psi = initObj.psi
      }
      else {
        this.psi = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type sysinfo
    // Serialize message field [header]
    bufferOffset = _serializer.float64(obj.header, buffer, bufferOffset);
    // Serialize message field [cputemp]
    bufferOffset = _serializer.float64(obj.cputemp, buffer, bufferOffset);
    // Serialize message field [cpuload]
    bufferOffset = _serializer.float64(obj.cpuload, buffer, bufferOffset);
    // Serialize message field [freeram]
    bufferOffset = _serializer.float64(obj.freeram, buffer, bufferOffset);
    // Serialize message field [freehdd]
    bufferOffset = _serializer.float64(obj.freehdd, buffer, bufferOffset);
    // Serialize message field [uptime]
    bufferOffset = _serializer.float64(obj.uptime, buffer, bufferOffset);
    // Serialize message field [vbat]
    bufferOffset = _serializer.float64(obj.vbat, buffer, bufferOffset);
    // Serialize message field [rh]
    bufferOffset = _serializer.float64(obj.rh, buffer, bufferOffset);
    // Serialize message field [temp]
    bufferOffset = _serializer.float64(obj.temp, buffer, bufferOffset);
    // Serialize message field [psi]
    bufferOffset = _serializer.float64(obj.psi, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type sysinfo
    let len;
    let data = new sysinfo(null);
    // Deserialize message field [header]
    data.header = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [cputemp]
    data.cputemp = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [cpuload]
    data.cpuload = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [freeram]
    data.freeram = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [freehdd]
    data.freehdd = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [uptime]
    data.uptime = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [vbat]
    data.vbat = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [rh]
    data.rh = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [temp]
    data.temp = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [psi]
    data.psi = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 80;
  }

  static datatype() {
    // Returns string type for a message object
    return 'raspberrypi_vitals_msg/sysinfo';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'edf8691ad7d4ea111d25d08805390e3b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 header
    float64 cputemp
    float64 cpuload
    float64 freeram
    float64 freehdd
    float64 uptime
    float64 vbat
    float64 rh
    float64 temp
    float64 psi
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new sysinfo(null);
    if (msg.header !== undefined) {
      resolved.header = msg.header;
    }
    else {
      resolved.header = 0.0
    }

    if (msg.cputemp !== undefined) {
      resolved.cputemp = msg.cputemp;
    }
    else {
      resolved.cputemp = 0.0
    }

    if (msg.cpuload !== undefined) {
      resolved.cpuload = msg.cpuload;
    }
    else {
      resolved.cpuload = 0.0
    }

    if (msg.freeram !== undefined) {
      resolved.freeram = msg.freeram;
    }
    else {
      resolved.freeram = 0.0
    }

    if (msg.freehdd !== undefined) {
      resolved.freehdd = msg.freehdd;
    }
    else {
      resolved.freehdd = 0.0
    }

    if (msg.uptime !== undefined) {
      resolved.uptime = msg.uptime;
    }
    else {
      resolved.uptime = 0.0
    }

    if (msg.vbat !== undefined) {
      resolved.vbat = msg.vbat;
    }
    else {
      resolved.vbat = 0.0
    }

    if (msg.rh !== undefined) {
      resolved.rh = msg.rh;
    }
    else {
      resolved.rh = 0.0
    }

    if (msg.temp !== undefined) {
      resolved.temp = msg.temp;
    }
    else {
      resolved.temp = 0.0
    }

    if (msg.psi !== undefined) {
      resolved.psi = msg.psi;
    }
    else {
      resolved.psi = 0.0
    }

    return resolved;
    }
};

module.exports = sysinfo;
