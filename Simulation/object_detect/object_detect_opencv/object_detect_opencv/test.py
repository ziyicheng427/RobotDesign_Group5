import pyrealsense2 as rs
pipe = rs.pipeline
pipe_profile = pipe.start()

pipe_profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()