import os


def clean_up_hls_stream(name: str, output: str):
  """
  Removes all references to the HLS playlist for the
  given stream name
  """
  files = filter(lambda f: f.startswith(name), os.listdir(output))
  for file_name in files:
    os.remove(os.path.join(output, file_name))


def get_esp32_encoding_pipeling(config: dict, output_directory: str):
  stream_file = os.path.join(output_directory, config.get('stream_name')) + "_%s.mp4"
  file_pipe = f"[f=segment\:segment_time={config.get('segment_time')}\:reset_timestamps=1\:strftime=1]{stream_file}|"
  udp_pipe = f"[f=mpegts]udp\://{config.get('stream_host')}\:{config.get('stream_port')}/"
  video_filter = ""

  if not config.get('archive'):
    file_pipe = ""
  
  if config.get('grayscale'):
    video_filter = "-vf format=gray "

  return f"""ffmpeg \
    -an \
    -framerate {config.get('infps')} \
    -video_size {config.get('inres')} \
    -i http://{config.get('ip_address')}:81/stream \
    -f mpegts \
    -vcodec mpeg1video {video_filter}\
    -threads {config.get('threads')} \
    -vsync {config.get('vsync')} \
    -b:v {config.get('bitrate')} \
    -s {config.get('outres')} \
    -r {config.get('outfps')} \
    -bf 0 \
    -f tee -map 0:v "{file_pipe}{udp_pipe}"
  """


def get_pi_usb_encoding_pipeline(config: dict, output_directory: str):
  
  #-vf "crop=iw-mod(iw\,2):ih-mod(ih\,2)"
  
  stream_file = os.path.join(output_directory, config.get('stream_name')) + "_%s.mp4"
  file_pipe = f"[f=segment\:segment_time={config.get('segment_time')}\:reset_timestamps=1\:strftime=1]{stream_file}|"
  udp_pipe = f"[f=mpegts]udp\://{config.get('stream_host')}\:{config.get('stream_port')}/"
  video_filter = ""

  if not config.get('archive'):
    file_pipe = ""
  
  if config.get('grayscale'):
    video_filter = "-vf format=gray "

  return f"""ffmpeg \
    -an \
    -f v4l2 \
    -framerate {config.get('infps')} \
    -video_size {config.get('inres')} \
    -i /dev/video{config.get('video_index')} \
    -f mpegts \
    -vcodec mpeg1video {video_filter}\
    -threads {config.get('threads')} \
    -vsync {config.get('vsync')} \
    -b:v {config.get('bitrate')} \
    -s {config.get('outres')} \
    -r {config.get('outfps')} \
    -bf 0 \
    -f tee -map 0:v "{file_pipe}{udp_pipe}"
  """


def get_mac_webcam_encoding_pipeline(config: dict, output_directory: str):
  stream_file = os.path.join(output_directory, config.get('stream_name')) + "_%s.mp4"
  file_pipe = f"[f=segment\:segment_time={config.get('segment_time')}\:reset_timestamps=1\:strftime=1]{stream_file}|"
  udp_pipe = f"[f=mpegts]udp\://{config.get('stream_host')}\:{config.get('stream_port')}/"
  video_filter = ""

  if not config.get('archive'):
    file_pipe = ""
  
  if config.get('grayscale'):
    video_filter = "-vf format=gray "

  return f"""ffmpeg \
    -re \
    -an \
    -f avfoundation \
    -pix_fmt yuyv422 \
    -framerate {config.get('infps')} \
    -video_size {config.get('inres')} \
    -i "0:0" \
    -vcodec mpeg1video {video_filter}\
    -vsync {config.get('vsync')} \
    -threads {config.get('threads')} \
    -b:v {config.get('bitrate')} \
    -s {config.get('outres')} \
    -r {config.get('outfps')} \
    -bf 0 \
    -f tee \
    -map 0:v "{file_pipe}{udp_pipe}"
  """


def get_hls_output_pipeline(options: dict, output_file: str):
    return f"""\
    -f hls \
    -hls_flags delete_segments \
    -hls_segment_type mpegts \
    -hls_allow_cache 0 \
    -hls_list_size {options.get('hls_list_size', 1)} \
    -hls_time {options.get('hls_time', 1)} \
    {output_file} \
    """