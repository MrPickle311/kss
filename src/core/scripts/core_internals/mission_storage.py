from data_models.kss_server import JsonMissionPackage, JsonMissionsMessage
from typing import Optional


class MissionStorage:
    def __init__(self):
        self._missions = JsonMissionsMessage(region_id='', missions=[])

    def insert_missions(self, new_missions: JsonMissionsMessage) -> None:
        self._missions = new_missions

    def get_next_mission(self) -> JsonMissionPackage:
        return self._missions.missions.pop(-1)

    def wipe_all_missions(self):
        self._missions.missions.clear()

    def is_empty(self) -> bool:
        return len(self._missions.missions) == 0
