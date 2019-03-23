// Auto-generated. Do not edit!

// (in-package quadrotor_state_machine.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class StateCommand {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.next = null;
      this.current = null;
    }
    else {
      if (initObj.hasOwnProperty('next')) {
        this.next = initObj.next
      }
      else {
        this.next = '';
      }
      if (initObj.hasOwnProperty('current')) {
        this.current = initObj.current
      }
      else {
        this.current = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type StateCommand
    // Serialize message field [next]
    bufferOffset = _serializer.string(obj.next, buffer, bufferOffset);
    // Serialize message field [current]
    bufferOffset = _serializer.string(obj.current, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type StateCommand
    let len;
    let data = new StateCommand(null);
    // Deserialize message field [next]
    data.next = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [current]
    data.current = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.next.length;
    length += object.current.length;
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'quadrotor_state_machine/StateCommand';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '928bf34b732b7e32d1524eb9acbd1715';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string next
    string current
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new StateCommand(null);
    if (msg.next !== undefined) {
      resolved.next = msg.next;
    }
    else {
      resolved.next = ''
    }

    if (msg.current !== undefined) {
      resolved.current = msg.current;
    }
    else {
      resolved.current = ''
    }

    return resolved;
    }
};

module.exports = StateCommand;
