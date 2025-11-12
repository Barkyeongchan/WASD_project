# wasd_interfaces/msg/TaskState.msg

string task_id      # 어떤 작업에 대한 상태인지
uint8 state
string message      # 디버깅용 / UI에 보여줄 짧은 설명

# state 값 의미 정의
uint8 STATE_UNKNOWN         = 0
uint8 STATE_RECEIVED        = 1   # 요청 들어옴
uint8 STATE_MOVING_TO_RACK  = 2   # 랙으로 이동 중
uint8 STATE_ARRIVED_AT_RACK = 3   # 랙 도착 (작업자가 적재하는 단계)
uint8 STATE_RETURNING_HOME  = 4   # 복귀 중
uint8 STATE_DONE            = 5   # 전체 완료
uint8 STATE_ERROR           = 100 # 에러