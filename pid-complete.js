let clamp = function(val, low, high) {
  if (val > high) return high;
  if (val < low) return low;
  return val;
}

module.exports = function(RED) {
    function CompletePIDNode(config) {
        RED.nodes.createNode(this,config);
        var node = this;

        node.sampleSec = parseFloat(config.sampleInterval) || 1;
        node.sampleMS = node.sampleSec * 1000;
        node.outMin = parseFloat(config.outMin) || 0;
        node.outMax = parseFloat(config.outMax) || 1;
        node.kp = parseFloat(config.kp) || 1;
        node.ki = parseFloat(config.ki) || 0.01;
        node.kd = parseFloat(config.kd) || 0;
        node.sp = parseFloat(config.sp) || 100;
        node.PonM = config.PonM || false;
        node.DonM = config.DonM || false;
        node.bumpPercent = parseFloat(config.bumpPercent) || 1;

        node.enabled = false;
        node.auto = false;

        function reset() {
          node._proportional = 0;
          node._integral = 0;
          node._derivative = 0;

          node._lastTime = Date.now();
          node._lastInput = null;
          node._lastOutput = null;
          node._lastError = null;
        }

        function setAuto(output = null) {
          if (!node.enabled) return;

          if (!node.auto) {
            reset();
            node._integral = output ||  0;
            node._integral = clamp(node._integral, node.outMin, node.outMax);
            node.auto = true;
          }
        }

        function setManual(output) {
          if (!node.enabled) return;

          reset();
          node._lastOutput = output || node._lastOutput;
          node._lastOutput = clamp(node._lastOutput, node.outMin, node.outMax);
          node.auto = false;

        }

        function calculate(input) {
          let now = Date.now();
          let dt = now - (node._lastTime || 1e-16);
          if (dt < node.sampleMS) return node._lastOutput;

          let error = node.sp - input;
          let dInput = input - (node._lastInput || input);
          let dError = error - (node._lastError || error);

          if (node.PonM) {
            node._proportional -= node.kp * dInput;
          } else {
            node._proportional = node.kp * error;
          }

          node._integral += node.ki * error * dt;
          node._integral = clamp(node._integral, node.outMin, node.outMax);

          if (node.DonM) {
            node._derivative = -1 * node.kd * dInput;
          } else {
            node._derivative = node.kd * error;
          }

          let output = node._proportional + node._integral + node._derivative;
          output = clamp(output, node.outMin, node.outMax);

          node._lastTime = now;
          node._lastOutput = output;
          node._lasInput = input;
          node._lastError = error;
        }

        reset();

        node.on('input', function(msg, send, done) {
          let newMsg = {};
          if (msg.hasOwnProperty("cmd")) {
            if (msg.cmd === "reset") {
              reset();
            } else if (msg.cmd === "disable") {
              node.enabled = false;
            } else if (msg.cmd === "auto") {
              node.enabled = true;
              setAuto();
            } else if (msg.cmd === "manual") {
              node.enabled = true;
              let output = null;
              if (msg.hasOwnProperty("payload")) {
                output = parseFloat(msg.payload);
              }
              setManual(output);
              newMsg.payload = node._lastOutput;
            }
          } else if (msg.hasOwnProperty("payload")) {
            if (node.enabled) {
              let tempOutput = parseFloat(msg.payload);
              if (node.auto) {
                if (!isNaN(tempOutput)) {
                  calculate(tempOutput);
                  newMsg.proportional = node._proportional;
                  newMsg.integral = node._integral;
                  newMsg.derivative = node._derivative;
                }
              }
              newMsg.payload = node._lastOutput;
            }
          }
          send(newMsg);
          if (done) done();
        });
    }
    RED.nodes.registerType("pid-complete",CompletePIDNode);
}
