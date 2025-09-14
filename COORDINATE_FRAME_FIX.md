# 좌표계 정렬 문제 해결 방안

## 문제 분석

### 현재 상황
1. **초기 방향 불일치**: 차량이 가제보에서 북서쪽(~140°) 방향을 바라보며 시작하지만, RViz2의 좌표계 축들(map, odom, base_footprint)이 모두 같은 방향으로 정렬되어 140° 오프셋이 반영되지 않음
2. **오도메트리 방향 불일치**: RViz2에서 odometry 화살표가 차량의 실제 이동 방향과 일치하지 않음
3. **IMU + 휠 오도메트리 통합 후 정확도 저하**: GPS만 사용했을 때보다 정확도가 떨어짐

### ENU 좌표계 이해
- **ENU (East-North-Up)**: 동쪽(E) = +X축(빨간색), 북쪽(N) = +Y축(초록색), 위쪽(U) = +Z축(파란색)
- **RViz2에서**: 빨간축이 동쪽, 초록축이 북쪽을 의미하는 것이 맞습니다
- **차량 좌표계**: base_footprint의 빨간축(+X)이 차량의 전진 방향

## 해결 방안

### 1. 좌표계 정렬 수정
**문제**: 차량이 북서쪽(140°)을 바라보지만 좌표계가 이를 반영하지 않음

**해결책**:
```python
# 추가된 정적 TF 변환 (gazebo_model.launch.py)
tf_map_orientation_correction = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='tf_map_orientation_correction',
    arguments=[
        '0', '0', '0',
        '0', '0', '2.8',  # 차량 스폰 yaw와 동일한 오프셋 적용
        'map_raw',
        'map'
    ],
)
```

### 2. GPS 가중치 증가
**문제**: IMU + 휠 오도메트리 통합 후 정확도 저하

**해결책**:
- EKF 로컬 파라미터에서 휠 오도메트리 프로세스 노이즈 증가
- EKF 글로벌 파라미터에서 GPS 위치 프로세스 노이즈 감소
- GPS 공분산 개선 (0.15m → 0.1m)

```yaml
# ekf_local.yaml
process_noise_covariance_diagonal:
  [0.1,  0.1,  1.0,    # 위치 - 증가된 노이즈
   0.05, 0.05, 0.05,   # 회전
   1.0,  1.0,  1.0,    # 속도 - 증가된 노이즈
   0.2,  0.2,  0.3,    # 각속도
   1.0,  1.0,  1.0]    # 가속도

# ekf_global.yaml  
process_noise_covariance_diagonal:
  [0.01, 0.01, 1.0,    # 위치 - GPS를 더 신뢰하도록 감소
   0.05, 0.05, 0.05,
   0.5,  0.5,  1.0,
   0.1,  0.1,  0.2,
   0.5,  0.5,  1.0]
```

### 3. 좌표계 체인 구조
```
utm → map_raw → map → odom → base_footprint
```

- **utm**: UTM 좌표계 (GPS 기반)
- **map_raw**: 회전 보정 전 맵 좌표계
- **map**: 차량 초기 방향에 맞춰 회전 보정된 맵 좌표계
- **odom**: 로컬 오도메트리 좌표계
- **base_footprint**: 차량 베이스 좌표계

### 4. 디버그 및 검증
새로운 디버그 노드(`frame_alignment_debug.py`)가 추가되어:
- 각 좌표계의 축 방향을 화살표로 시각화
- 좌표계 간 변환 정보를 로그로 출력
- 실시간으로 좌표계 정렬 상태 모니터링

## 사용법

### 1. 시뮬레이션 실행
```bash
ros2 launch mobile_robot gazebo_model.launch.py
```

### 2. RViz2에서 확인사항
- Fixed Frame을 `map`으로 설정
- Axes 표시에서 다음 프레임들 추가:
  - `map` (빨간 화살표가 동쪽)
  - `odom` 
  - `base_footprint` (빨간 화살표가 차량 전진 방향)
- `/frame_debug_markers` 토픽 추가하여 시각화된 좌표축 확인
- `/odometry/filtered_map` 토픽으로 오도메트리 확인

### 3. 좌표계 정렬 확인
- 차량의 base_footprint 빨간축이 차량이 바라보는 방향(북서쪽)을 가리켜야 함
- odometry 화살표가 차량 이동 방향과 일치해야 함
- GPS 위치와 오도메트리 위치가 정확히 일치해야 함

## 주요 변경사항 요약

1. **EKF 파라미터 조정**: GPS 가중치 증가, 휠 오도메트리 가중치 감소
2. **좌표계 보정**: map_raw → map 변환으로 초기 방향 오프셋 적용
3. **GPS 정확도 향상**: 공분산 감소 (0.15m → 0.1m)
4. **디버그 도구 추가**: 실시간 좌표계 시각화 및 모니터링
5. **navsat 설정 업데이트**: map_raw 프레임 사용으로 변환 체인 정리

이러한 변경으로 차량의 실제 방향과 좌표계가 정확히 일치하고, GPS 기반의 정확한 위치 추정이 가능해집니다.