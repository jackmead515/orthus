const http = require('http');
const fs = require('fs');
const path = require('path');

const socket = require('./socket');
const config = require('./config');

async function main() {
    console.info('loaded configurations', JSON.stringify(config));
    
    const httpServer = http.createServer();

    await socket.initialize(httpServer);

    httpServer.listen(config.httpPort, () => {
        console.log('relay started');
    });
}

main();