from src.djikmz import DroneTask

mission = (DroneTask("M3E", "Python")
           .name("Test Video Recording")
           .payload("M3E")
           .altitude(50.0)
           .speed(10.0)
           .return_home_on_signal_loss(True)

            .fly_to(37.7749, -122.4194)
                .start_video_recording()
                .gimbal_rotate(-30, 10, 0)
                .set_focus(True,0.5,0.5, is_infinite=True)
                .hover(10)
            .fly_to(37.7750, -122.4184)
                .stop_video_recording()
           )

kml = mission.build()

xml = kml.to_xml()
with open("test_video_recording.kml", "w") as f:
    f.write(xml)