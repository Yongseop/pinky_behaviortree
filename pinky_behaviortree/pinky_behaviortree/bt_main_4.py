def make_bt():
    root = Selector(name="Root", memory=True)

    # Navigation sequence
    nav_sequence = Sequence(
        name="NavigationSequence",
        memory=True,
        children=[
            NavigateToGoal()
        ]
    )
    root.add_child(nav_sequence)

    # Recovery sequence
    rotate_detect_sequence = Sequence(
        name="RotateDetectSequence",
        memory=True,
        children=[
            Sequence(  # 회전 및 정지를 위한 하위 시퀀스
                name="RotateStopSequence",
                memory=True,
                children=[
                    RotateRobot(rotation_time=3.0),  # 3초 회전
                    StopRobot(stop_time=1.0)         # 1초 정지
                ]
            ),
            Sequence(  # 마커 감지 및 위치 보정을 위한 하위 시퀀스
                name="DetectLocalizeSequence",
                memory=True,
                children=[
                    DetectMarker(detection_time=2.0),  # 2초 동안 마커 감지
                    ArucoLocalization()
                ]
            )
        ]
    )
    rotation_counter = Counter(child=rotate_detect_sequence, max_count=10)
    root.add_child(rotation_counter)

    return root