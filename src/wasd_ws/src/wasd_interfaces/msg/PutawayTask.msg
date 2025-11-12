# wasd_interfaces/msg/PutawayTask.msg

string task_id      # "20251111-001" 같이 서버에서 만들어도 되고, 그냥 "" 도 가능
string item_type    # 예: "A", "B", "BOX_SMALL"
int32 quantity      # 수량, 0 이상