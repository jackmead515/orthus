import React from 'react';
import eio from 'engine.io-client';

class JSMpegWritableSource {
  constructor(url, options) {
    this.destination = null;
    this.completed = false;
    this.established = false;
    this.progress = 0;
    this.streaming = true;
  }

  connect(destination) {
    this.destination = destination;
  }

  start() {
    this.established = true;
    this.completed = true;
    this.progress = 1;
  }

  resume() {}

  destroy() {}

  write(data) {
    this.destination.write(data);
  }
}

export default class App extends React.PureComponent {

  constructor(props) {
    super(props);

    this.state = {
      connected: false,
      streaming: false,
      avgPacketSize: 0,
    };

    this.socket = undefined;
    this.player = undefined;
    this.streamingTimeout = undefined;

    this.packetSizes = [];
    this.packetSizeInterval = undefined;
  }

  componentWillUnmount() {
    if (this.socket) {
      this.socket.close();
    }

    if (this.packetSizeInterval) {
      clearInterval(this.packetSizeInterval);
    }
  }

  componentDidMount() {
    const host = window.location.hostname;

    this.socket = new eio.Socket(`ws://${host}:5454`, {
			upgrade: true,
			//transports: ['websocket']
		});
		this.socket.binaryType = 'arraybuffer';
  
    const canvas = document.getElementById('video-canvas');

    // eslint-disable-next-line no-undef
		this.player = new JSMpeg.Player(this.socket, {
			source: JSMpegWritableSource,
			audio: false,
			canvas: canvas,
		});

    this.packetSizeInterval = setInterval(() => {
      let sum = 0;
      this.packetSizes.forEach((size) => sum += size);
      const avg = sum / this.packetSizes.length;
      this.setState({ avgPacketSize: avg });
    }, 1000);


		this.socket.on('open', () => {
      this.setState({ connected: true });

			this.socket.on('data', data => {
				if (!isNaN(data.byteLength)) {
          
          if (!this.state.streaming) {
            this.setState({ streaming: true });
            if (this.streamingTimeout) clearTimeout(this.streamingTimeout);
            this.streamingTimeout = setTimeout(() => this.setState({ streaming: false }), 5000);
          } else {
            if (this.streamingTimeout) clearTimeout(this.streamingTimeout);
            this.streamingTimeout = setTimeout(() => this.setState({ streaming: false }), 5000);
          }

          this.packetSizes.push(data.byteLength);
          if (this.packetSizes.length > 100) {
            this.packetSizes.shift();
          }

          try {
            this.player.source.write(data);
          } catch (e) {}
				}
			});
  
		});
		this.socket.on('close', (error) => {
      if (this.streamingTimeout) clearTimeout(this.streamingTimeout);
      this.setState({ connected: false, streaming: false });
		})
		this.socket.on('error', (error) => {
			console.log('error', error);
		});
  }


  startStream() {
    const host = window.location.hostname;
    const url = `http://${host}:8000/api/stream`;

    // post request to start stream
    //fetch(url, { method: 'POST' });

    this.socket.send('play_stream');
  }

  killStream() {
    const host = window.location.hostname;
    const url = `http://${host}:8000/api/stream`;

    // post request to start stream
    //fetch(url, { method: 'DELETE' });

    this.socket.send('stop_stream');
  }

  renderStreaming() {
    if (this.state.streaming) {
      return (
        <div class="connected">
          Streaming
          <div class='connected--true'></div>
        </div>
      )
    }

    return (
      <div class="connected">
        Not Streaming
        <div class='connected--false'></div>
      </div>
    )
  }

  renderConnected() {
    if (this.state.connected) {
      return (
        <div class="connected">
          Connected
          <div class='connected--true'></div>
        </div>
      )
    }

    return (
      <div class="connected">
        Disconnected
        <div class='connected--false'></div>
      </div>
    )
  }

  render() {
    return (
      <div className='content'>
        <canvas className='video' id="video-canvas">
          
        </canvas>
        <div className="controls">
          <div className="controls--buttons">
            <button onClick={() => this.startStream()}>Stream and Record</button>
            <button onClick={() => this.killStream()}>Stop</button>
          </div>
          <div className="controls--status">
            {this.renderConnected()}
            {this.renderStreaming()}
            {this.state.avgPacketSize} bytes
          </div>
        </div>
      </div>
    );
  }
}
