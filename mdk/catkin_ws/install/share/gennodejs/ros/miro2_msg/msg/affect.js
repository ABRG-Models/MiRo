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

class affect {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.valence = null;
      this.arousal = null;
    }
    else {
      if (initObj.hasOwnProperty('valence')) {
        this.valence = initObj.valence
      }
      else {
        this.valence = 0.0;
      }
      if (initObj.hasOwnProperty('arousal')) {
        this.arousal = initObj.arousal
      }
      else {
        this.arousal = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type affect
    // Serialize message field [valence]
    bufferOffset = _serializer.float32(obj.valence, buffer, bufferOffset);
    // Serialize message field [arousal]
    bufferOffset = _serializer.float32(obj.arousal, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type affect
    let len;
    let data = new affect(null);
    // Deserialize message field [valence]
    data.valence = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [arousal]
    data.arousal = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'miro2_msg/affect';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b9db7d9709bc98cc560b83ce0bc4f004';
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
    #
    #	The "affect" state is two-dimensional, encoding valence
    #	(-ve = sad, +ve = happy) and arousal (-ve = relaxed, +ve = alert).
    #	The states are usually driven by signals entering the robot's
    #	sensory systems, but can also be driven directly by other systems.
    
    float32 valence
    float32 arousal
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new affect(null);
    if (msg.valence !== undefined) {
      resolved.valence = msg.valence;
    }
    else {
      resolved.valence = 0.0
    }

    if (msg.arousal !== undefined) {
      resolved.arousal = msg.arousal;
    }
    else {
      resolved.arousal = 0.0
    }

    return resolved;
    }
};

module.exports = affect;
