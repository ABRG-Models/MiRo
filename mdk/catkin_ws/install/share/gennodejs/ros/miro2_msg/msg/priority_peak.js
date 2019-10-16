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

class priority_peak {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.stream_index = null;
      this.loc_d = null;
      this.height = null;
      this.size = null;
      this.azim = null;
      this.elev = null;
      this.size_norm = null;
      this.volume = null;
      this.range = null;
      this.actioned = null;
    }
    else {
      if (initObj.hasOwnProperty('stream_index')) {
        this.stream_index = initObj.stream_index
      }
      else {
        this.stream_index = 0;
      }
      if (initObj.hasOwnProperty('loc_d')) {
        this.loc_d = initObj.loc_d
      }
      else {
        this.loc_d = new Array(2).fill(0);
      }
      if (initObj.hasOwnProperty('height')) {
        this.height = initObj.height
      }
      else {
        this.height = 0.0;
      }
      if (initObj.hasOwnProperty('size')) {
        this.size = initObj.size
      }
      else {
        this.size = 0.0;
      }
      if (initObj.hasOwnProperty('azim')) {
        this.azim = initObj.azim
      }
      else {
        this.azim = 0.0;
      }
      if (initObj.hasOwnProperty('elev')) {
        this.elev = initObj.elev
      }
      else {
        this.elev = 0.0;
      }
      if (initObj.hasOwnProperty('size_norm')) {
        this.size_norm = initObj.size_norm
      }
      else {
        this.size_norm = 0.0;
      }
      if (initObj.hasOwnProperty('volume')) {
        this.volume = initObj.volume
      }
      else {
        this.volume = 0.0;
      }
      if (initObj.hasOwnProperty('range')) {
        this.range = initObj.range
      }
      else {
        this.range = 0.0;
      }
      if (initObj.hasOwnProperty('actioned')) {
        this.actioned = initObj.actioned
      }
      else {
        this.actioned = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type priority_peak
    // Serialize message field [stream_index]
    bufferOffset = _serializer.int32(obj.stream_index, buffer, bufferOffset);
    // Check that the constant length array field [loc_d] has the right length
    if (obj.loc_d.length !== 2) {
      throw new Error('Unable to serialize array field loc_d - length must be 2')
    }
    // Serialize message field [loc_d]
    bufferOffset = _arraySerializer.float32(obj.loc_d, buffer, bufferOffset, 2);
    // Serialize message field [height]
    bufferOffset = _serializer.float32(obj.height, buffer, bufferOffset);
    // Serialize message field [size]
    bufferOffset = _serializer.float32(obj.size, buffer, bufferOffset);
    // Serialize message field [azim]
    bufferOffset = _serializer.float32(obj.azim, buffer, bufferOffset);
    // Serialize message field [elev]
    bufferOffset = _serializer.float32(obj.elev, buffer, bufferOffset);
    // Serialize message field [size_norm]
    bufferOffset = _serializer.float32(obj.size_norm, buffer, bufferOffset);
    // Serialize message field [volume]
    bufferOffset = _serializer.float32(obj.volume, buffer, bufferOffset);
    // Serialize message field [range]
    bufferOffset = _serializer.float32(obj.range, buffer, bufferOffset);
    // Serialize message field [actioned]
    bufferOffset = _serializer.int32(obj.actioned, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type priority_peak
    let len;
    let data = new priority_peak(null);
    // Deserialize message field [stream_index]
    data.stream_index = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [loc_d]
    data.loc_d = _arrayDeserializer.float32(buffer, bufferOffset, 2)
    // Deserialize message field [height]
    data.height = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [size]
    data.size = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [azim]
    data.azim = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [elev]
    data.elev = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [size_norm]
    data.size_norm = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [volume]
    data.volume = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [range]
    data.range = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [actioned]
    data.actioned = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 44;
  }

  static datatype() {
    // Returns string type for a message object
    return 'miro2_msg/priority_peak';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd2924675ffe77da16f66ce7eb1cbeb6d';
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
    #	For a full copy of the license agreement, and a complete
    #	definition of "The Software", see LICENSE in the MDK root
    #	directory.
    #	
    #	Subject to the terms of this Agreement, Consequential
    #	Robotics grants to you a limited, non-exclusive, non-
    #	transferable license, without right to sub-license, to use
    #	"The Software" in accordance with this Agreement and any
    #	other written agreement with Consequential Robotics.
    #	Consequential Robotics does not transfer the title of "The
    #	Software" to you; the license granted to you is not a sale.
    #	This agreement is a binding legal agreement between
    #	Consequential Robotics and the purchasers or users of "The
    #	Software".
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
    
    int32 stream_index
    float32[2] loc_d
    float32 height
    float32 size
    float32 azim
    float32 elev
    
    float32 size_norm
    float32 volume
    float32 range
    
    int32 actioned
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new priority_peak(null);
    if (msg.stream_index !== undefined) {
      resolved.stream_index = msg.stream_index;
    }
    else {
      resolved.stream_index = 0
    }

    if (msg.loc_d !== undefined) {
      resolved.loc_d = msg.loc_d;
    }
    else {
      resolved.loc_d = new Array(2).fill(0)
    }

    if (msg.height !== undefined) {
      resolved.height = msg.height;
    }
    else {
      resolved.height = 0.0
    }

    if (msg.size !== undefined) {
      resolved.size = msg.size;
    }
    else {
      resolved.size = 0.0
    }

    if (msg.azim !== undefined) {
      resolved.azim = msg.azim;
    }
    else {
      resolved.azim = 0.0
    }

    if (msg.elev !== undefined) {
      resolved.elev = msg.elev;
    }
    else {
      resolved.elev = 0.0
    }

    if (msg.size_norm !== undefined) {
      resolved.size_norm = msg.size_norm;
    }
    else {
      resolved.size_norm = 0.0
    }

    if (msg.volume !== undefined) {
      resolved.volume = msg.volume;
    }
    else {
      resolved.volume = 0.0
    }

    if (msg.range !== undefined) {
      resolved.range = msg.range;
    }
    else {
      resolved.range = 0.0
    }

    if (msg.actioned !== undefined) {
      resolved.actioned = msg.actioned;
    }
    else {
      resolved.actioned = 0
    }

    return resolved;
    }
};

module.exports = priority_peak;
