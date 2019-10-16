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

class object_face {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.conf = null;
      this.corner = null;
      this.size = null;
    }
    else {
      if (initObj.hasOwnProperty('conf')) {
        this.conf = initObj.conf
      }
      else {
        this.conf = 0.0;
      }
      if (initObj.hasOwnProperty('corner')) {
        this.corner = initObj.corner
      }
      else {
        this.corner = new Array(2).fill(0);
      }
      if (initObj.hasOwnProperty('size')) {
        this.size = initObj.size
      }
      else {
        this.size = new Array(2).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type object_face
    // Serialize message field [conf]
    bufferOffset = _serializer.float32(obj.conf, buffer, bufferOffset);
    // Check that the constant length array field [corner] has the right length
    if (obj.corner.length !== 2) {
      throw new Error('Unable to serialize array field corner - length must be 2')
    }
    // Serialize message field [corner]
    bufferOffset = _arraySerializer.float32(obj.corner, buffer, bufferOffset, 2);
    // Check that the constant length array field [size] has the right length
    if (obj.size.length !== 2) {
      throw new Error('Unable to serialize array field size - length must be 2')
    }
    // Serialize message field [size]
    bufferOffset = _arraySerializer.float32(obj.size, buffer, bufferOffset, 2);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type object_face
    let len;
    let data = new object_face(null);
    // Deserialize message field [conf]
    data.conf = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [corner]
    data.corner = _arrayDeserializer.float32(buffer, bufferOffset, 2)
    // Deserialize message field [size]
    data.size = _arrayDeserializer.float32(buffer, bufferOffset, 2)
    return data;
  }

  static getMessageSize(object) {
    return 20;
  }

  static datatype() {
    // Returns string type for a message object
    return 'miro2_msg/object_face';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0608d5dd7e088755bafb29283da99f24';
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
    
    float32 conf
    float32[2] corner
    float32[2] size
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new object_face(null);
    if (msg.conf !== undefined) {
      resolved.conf = msg.conf;
    }
    else {
      resolved.conf = 0.0
    }

    if (msg.corner !== undefined) {
      resolved.corner = msg.corner;
    }
    else {
      resolved.corner = new Array(2).fill(0)
    }

    if (msg.size !== undefined) {
      resolved.size = msg.size;
    }
    else {
      resolved.size = new Array(2).fill(0)
    }

    return resolved;
    }
};

module.exports = object_face;
