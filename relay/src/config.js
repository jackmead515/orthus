
module.exports = {
    environment: process.env.NODE_ENV || 'local',
    streamPort: process.env.STREAM_PORT || 3131,
    streamHost: process.env.STREAM_HOST || '127.0.0.1',
    httpPort: process.env.HTTP_PORT || 5454,
    httpHost: process.env.HTTP_HOST || '127.0.0.1',
    maxWriteBuffer: process.env.MAX_WRITE_BUFFER || 100
}