const engine = require('engine.io');
const dgram = require('dgram');

const config = require('./config');
const conflator = require('./conflator');

let eioServer;
let relay = null;

function broadcast(message) {
    for (const sid in eioServer.clients) {
        const client = eioServer.clients[sid];
        if (client.state && client.state.streamRequested) {
            client.send(message);
        }
    }
}

function streamIsRequested() {
    for (const sid in eioServer.clients) {
        const client = eioServer.clients[sid];
        if (client.state && client.state.streamRequested) {
            return true;
        }
    }
    return false;
}

function onVideoMessage() {
    return (packet) => {
        broadcast(packet);
    }
}

async function createRelay() {
    const server = dgram.createSocket('udp4');
    const callback = onVideoMessage();

    return await new Promise((resolve) => {
        server.bind(config.streamPort, config.streamHost, () => {
            return resolve({
                server,
                callback
            });
        })
    });
}

function toggleRelay() {
    const { server, callback } = relay;
    const hasListener = server.eventNames().includes('message');
    const streamRequested = streamIsRequested();
    if (streamRequested && !hasListener) {
        console.log('attaching relay for stream');
        server.addListener('message', callback);
    } else if (!streamRequested && hasListener) {
        console.log('removing relay for stream');
        server.removeListener('message', callback);
    }
}

function onConnection(socket) {
    console.log('client connected');

    socket.state = {
        streamRequested: false,
    };

    socket.on('message', (message) => {

        if (message === 'play_stream') {
            console.log('client requesting stream');
            socket.state.streamRequested = true;
            toggleRelay();
        } else if (message === 'stop_stream') {
            console.log('client stopping stream');
            socket.state.streamRequested = false;
            toggleRelay();
        }

    });

    socket.on('close', () => {
        if (eioServer.clients[socket.sid]) {
            delete eioServer.clients[socket.sid];
        }
        console.log('client disconnected');
    })
}

function conflate(socket, messages) {
    if (messages.length > config.maxWriteBuffer) {
        console.log('conflating messages');
        return [];
    }

    return messages;
}

async function initialize(httpServer) {
    eioServer = engine.attach(httpServer, {
        pingTimeout: 2000,
        pingInterval: 10000,
        allowEIO3: true,
        httpCompression: false,
        cors: {
            credentials: true,
            origin: (_, cb) => cb(null, true),
        }
    });

    eioServer.on('connection', onConnection);
    eioServer.on('flush', conflator(conflate));

    relay = await createRelay();

    setInterval(toggleRelay, 1000);
}

module.exports = {
    initialize,
    broadcast,
    streamIsRequested
}