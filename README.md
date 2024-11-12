# Orthus

A stereographic calibration and streaming tool for underwater cameras.

## Features

- **Interactive Calibration**: Calibrate a stereo camera with live feedback straight in the web browser. You can view the captured chessboard patterns, and run the calibration algorithm in parallel to get the best results.

- **Web Browser Streaming**: Stream the stereo camera feed to a web browser utilizing JSMPEG. No need for installation of any client side software, just open the browser and start streaming.

## Roadmap Checklist

### Core

- [x] Interactive calibration experience
- [x] parallel calibration algorithm
- [x] calibration review
    - [ ] toggle disparity map
    - [ ] toggle rectified images
- [ ] add command line arguments instead of yaml file
- [ ] add http server for toggles
- [ ] add streaming mode
    - [ ] add object tracking support
    - [ ] add object detection support

### Web Application

- [x] containerize web application
- [x] containerize web-socket server
- [x] web-socket server for streaming
- [x] web-application for streaming
- [x] backend api server to serve web-application
- [ ] add parameters for starting core service
- [ ] add api routes to start core service in sub-process
- [ ] UX to switch between modes (calibration, streaming)
- [ ] add api routes to update block matching parameters
- [ ] add block matching tuning to web-application
