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

    this.state = {};

    this.socket = undefined;
    this.player = undefined;
  }

  componentDidMount() {
    const host = window.location.hostname;

    this.socket = new eio.Socket(`ws://${host}:5454`, {
			upgrade: true,
			//transports: ['websocket']
		});
		this.socket.binaryType = 'arraybuffer';

    //this.socket = new WebSocket(`ws://${host}:5454`);
  
    const canvas = document.getElementById('video-canvas');

    // eslint-disable-next-line no-undef
		this.player = new JSMpeg.Player(this.socket, {
			source: JSMpegWritableSource,
			audio: false,
			canvas: canvas,
		});


		this.socket.on('open', () => {
			console.log('connected', this.socket);

			this.socket.on('data', data => {
        //console.log('data', data);
				if (!isNaN(data.byteLength)) {
					this.player.source.write(data);
				}
			});
  
		});
		this.socket.on('close', (error) => {
			console.log('disconnected', error);
		})
		this.socket.on('error', (error) => {
			console.log('error', error);
		});
  }

  playStream() {
    this.socket.send('play_stream');
  }

  pauseStream() {
    this.socket.send('stop_stream');
  }

  startStream() {
    const host = window.location.hostname;
    const url = `http://${host}:8000/api/stream`;

    // post request to start stream
    fetch(url, { method: 'POST' });

    this.socket.send('play_stream');
  }

  killStream() {
    const host = window.location.hostname;
    const url = `http://${host}:8000/api/stream`;

    // post request to start stream
    fetch(url, { method: 'DELETE' });

    this.socket.send('stop_stream');
  }

  render() {
    return (
      <div className='content'>
        <canvas className='video' id="video-canvas"></canvas>
        <div className="controls">
          <button onClick={() => this.playStream()}>Play</button>
          <button onClick={() => this.pauseStream()}>Pause</button>
          <button onClick={() => this.startStream()}>Start</button>
          <button onClick={() => this.killStream()}>Kill</button>

        </div>
      </div>
    );
  }
}
