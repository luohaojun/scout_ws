// Auto-generated. Do not edit!

// (in-package offb.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class obj {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.object = null;
      this.X_c = null;
      this.Y_c = null;
      this.Z_c = null;
    }
    else {
      if (initObj.hasOwnProperty('object')) {
        this.object = initObj.object
      }
      else {
        this.object = false;
      }
      if (initObj.hasOwnProperty('X_c')) {
        this.X_c = initObj.X_c
      }
      else {
        this.X_c = 0.0;
      }
      if (initObj.hasOwnProperty('Y_c')) {
        this.Y_c = initObj.Y_c
      }
      else {
        this.Y_c = 0.0;
      }
      if (initObj.hasOwnProperty('Z_c')) {
        this.Z_c = initObj.Z_c
      }
      else {
        this.Z_c = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type obj
    // Serialize message field [object]
    bufferOffset = _serializer.bool(obj.object, buffer, bufferOffset);
    // Serialize message field [X_c]
    bufferOffset = _serializer.float64(obj.X_c, buffer, bufferOffset);
    // Serialize message field [Y_c]
    bufferOffset = _serializer.float64(obj.Y_c, buffer, bufferOffset);
    // Serialize message field [Z_c]
    bufferOffset = _serializer.float64(obj.Z_c, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type obj
    let len;
    let data = new obj(null);
    // Deserialize message field [object]
    data.object = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [X_c]
    data.X_c = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [Y_c]
    data.Y_c = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [Z_c]
    data.Z_c = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 25;
  }

  static datatype() {
    // Returns string type for a message object
    return 'offb/obj';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '281cc9156c896ee80119925d1b2feeb6';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool object
    float64 X_c 
    float64 Y_c
    float64 Z_c
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new obj(null);
    if (msg.object !== undefined) {
      resolved.object = msg.object;
    }
    else {
      resolved.object = false
    }

    if (msg.X_c !== undefined) {
      resolved.X_c = msg.X_c;
    }
    else {
      resolved.X_c = 0.0
    }

    if (msg.Y_c !== undefined) {
      resolved.Y_c = msg.Y_c;
    }
    else {
      resolved.Y_c = 0.0
    }

    if (msg.Z_c !== undefined) {
      resolved.Z_c = msg.Z_c;
    }
    else {
      resolved.Z_c = 0.0
    }

    return resolved;
    }
};

module.exports = obj;
