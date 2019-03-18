// Auto-generated. Do not edit!

// (in-package miro2_msg.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class voice_state {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.breathing_phase = null;
      this.vocalising = null;
    }
    else {
      if (initObj.hasOwnProperty('breathing_phase')) {
        this.breathing_phase = initObj.breathing_phase
      }
      else {
        this.breathing_phase = 0.0;
      }
      if (initObj.hasOwnProperty('vocalising')) {
        this.vocalising = initObj.vocalising
      }
      else {
        this.vocalising = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type voice_state
    // Serialize message field [breathing_phase]
    bufferOffset = _serializer.float32(obj.breathing_phase, buffer, bufferOffset);
    // Serialize message field [vocalising]
    bufferOffset = _serializer.bool(obj.vocalising, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type voice_state
    let len;
    let data = new voice_state(null);
    // Deserialize message field [breathing_phase]
    data.breathing_phase = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [vocalising]
    data.vocalising = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 5;
  }

  static datatype() {
    // Returns string type for a message object
    return 'miro2_msg/voice_state';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e821a66f37dcfd027ec1d69a1734ae31';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #	@section COPYRIGHT
    #	Copyright (C) 2019 Consequential Robotics Ltd
    #	
    #	@section AUTHOR
    #	Consequential Robotics http://consequentialrobotics.com
    #	
    #	@section LICENSE
    #	For a full copy of the license agreement, see LICENSE in the
    #	MDK root directory.
    #	
    #	Subject to the terms of this Agreement, Consequential
    #	Robotics grants to you a limited, non-exclusive, non-
    #	transferable license, without right to sub-license, to use
    #	MIRO Developer Kit in accordance with this Agreement and any
    #	other written agreement with Consequential Robotics.
    #	Consequential Robotics does not transfer the title of MIRO
    #	Developer Kit to you; the license granted to you is not a
    #	sale. This agreement is a binding legal agreement between
    #	Consequential Robotics and the purchasers or users of MIRO
    #	Developer Kit.
    #	
    #	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY
    #	KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
    #	WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
    #	PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS
    #	OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR
    #	OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
    #	OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
    #	SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
    
    float32 breathing_phase
    bool vocalising
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new voice_state(null);
    if (msg.breathing_phase !== undefined) {
      resolved.breathing_phase = msg.breathing_phase;
    }
    else {
      resolved.breathing_phase = 0.0
    }

    if (msg.vocalising !== undefined) {
      resolved.vocalising = msg.vocalising;
    }
    else {
      resolved.vocalising = false
    }

    return resolved;
    }
};

module.exports = voice_state;
