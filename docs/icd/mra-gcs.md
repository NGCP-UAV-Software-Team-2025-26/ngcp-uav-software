# GCS-MRA Interface Control Document

| Message Name  | Sender | Receiver | Frequency | Data Fields | Example Values | Description |
| :-------------: |:------:| :--------: | :---------: | :-----------: | :--------------: | :-----------: |
| HEARTBEAT     | MRA    | GCS      | 1 Hz      | system_status (int), battery (%) | 1, 87 | Keeps link alive |
| SURVIVOR_FOUND| MRA    | GCS      | Event     | lat (float), lon (float), signal_strength (float) | 34.057, -117.821, 0.76 | Reports survivor location |
| DROP_REQUEST  | GCS    | MRA      | Event     | approved (bool) | True | GCS approves payload drop |
| DROP_CONFIRMED| MRA    | GCS      | Event     | timestamp (string), success (bool) | "12:04:32", True | Confirms package released |
