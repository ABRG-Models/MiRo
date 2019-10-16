// Auto-generated. Do not edit!

// (in-package miro2_msg.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let object_tag = require('./object_tag.js');
let object_ball = require('./object_ball.js');
let object_face = require('./object_face.js');

//-----------------------------------------------------------

class objects {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.stream_index = null;
      this.tags = null;
      this.balls = null;
      this.faces = null;
    }
    else {
      if (initObj.hasOwnProperty('stream_index')) {
        this.stream_index = initObj.stream_index
      }
      else {
        this.stream_index = 0;
      }
      if (initObj.hasOwnProperty('tags')) {
        this.tags = initObj.tags
      }
      else {
        this.tags = [];
      }
      if (initObj.hasOwnProperty('balls')) {
        this.balls = initObj.balls
      }
      else {
        this.balls = [];
      }
      if (initObj.hasOwnProperty('faces')) {
        this.faces = initObj.faces
      }
      else {
        this.faces = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type objects
    // Serialize message field [stream_index]
    bufferOffset = _serializer.int32(obj.stream_index, buffer, bufferOffset);
    // Serialize message field [tags]
    // Serialize the length for message field [tags]
    bufferOffset = _serializer.uint32(obj.tags.length, buffer, bufferOffset);
    obj.tags.forEach((val) => {
      bufferOffset = object_tag.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [balls]
    // Serialize the length for message field [balls]
    bufferOffset = _serializer.uint32(obj.balls.length, buffer, bufferOffset);
    obj.balls.forEach((val) => {
      bufferOffset = object_ball.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [faces]
    // Serialize the length for message field [faces]
    bufferOffset = _serializer.uint32(obj.faces.length, buffer, bufferOffset);
    obj.faces.forEach((val) => {
      bufferOffset = object_face.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type objects
    let len;
    let data = new objects(null);
    // Deserialize message field [stream_index]
    data.stream_index = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [tags]
    // Deserialize array length for message field [tags]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.tags = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.tags[i] = object_tag.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [balls]
    // Deserialize array length for message field [balls]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.balls = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.balls[i] = object_ball.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [faces]
    // Deserialize array length for message field [faces]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.faces = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.faces[i] = object_face.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 48 * object.tags.length;
    length += 16 * object.balls.length;
    length += 20 * object.faces.length;
    return length + 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'miro2_msg/objects';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0751afb21e15f80b448531e265b3963f';
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
    
    # header
    int32 stream_index
    
    #	objects
    object_tag[] tags
    object_ball[] balls
    object_face[] faces
    
    
    ================================================================================
    MSG: miro2_msg/object_tag
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
    int32 id
    float32[2] centre
    float32[8] corners
    
    
    ================================================================================
    MSG: miro2_msg/object_ball
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
    float32[2] centre
    float32 radius
    
    
    ================================================================================
    MSG: miro2_msg/object_face
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
    const resolved = new objects(null);
    if (msg.stream_index !== undefined) {
      resolved.stream_index = msg.stream_index;
    }
    else {
      resolved.stream_index = 0
    }

    if (msg.tags !== undefined) {
      resolved.tags = new Array(msg.tags.length);
      for (let i = 0; i < resolved.tags.length; ++i) {
        resolved.tags[i] = object_tag.Resolve(msg.tags[i]);
      }
    }
    else {
      resolved.tags = []
    }

    if (msg.balls !== undefined) {
      resolved.balls = new Array(msg.balls.length);
      for (let i = 0; i < resolved.balls.length; ++i) {
        resolved.balls[i] = object_ball.Resolve(msg.balls[i]);
      }
    }
    else {
      resolved.balls = []
    }

    if (msg.faces !== undefined) {
      resolved.faces = new Array(msg.faces.length);
      for (let i = 0; i < resolved.faces.length; ++i) {
        resolved.faces[i] = object_face.Resolve(msg.faces[i]);
      }
    }
    else {
      resolved.faces = []
    }

    return resolved;
    }
};

module.exports = objects;
