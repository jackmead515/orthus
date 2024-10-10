module.exports = function(conflater) {
    return function conflateOnFlush(socket, writeBuffer) {
      const nonMessages = [];
      let messages = [];
  
      for (let i = 0, l = writeBuffer.length; i < l; i++) {
        var current = writeBuffer[i];
        if (current.type === 'message') {
          messages.push(current);
        } else {
          nonMessages.push(current);
        }
      }
  
      if (messages.length > 0) {
        //{ type: 'message', data: msg }
        messages = conflater(socket, messages);
        writeBuffer.length = 0;
        Array.prototype.push.apply(writeBuffer, nonMessages);
        Array.prototype.push.apply(writeBuffer, messages);
      }
    };
};

