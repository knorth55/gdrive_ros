# gdrive_ros

Google Drive file uploader for ROS

## Usage

### Run server
```bash
rosrun gdrive_ros gdrive_server_node.py
```

### Call upload service

```bash
# single upload
rosservice call /gdrive_ros/upload ...
# multiple upload
rosservice call /gdrive_ros/upload_multi ...
```

## Parameters

- `~settings_yaml` (`string, default: `None`)

  - PyDrive settings yaml path

- `~share_type` (`string`, default: `anyone`)

  - Uploaded file share type

- `~share_value` (`string`, default: `anyone`)

  - Uploaded file share value

- `~share_role` (`string`, default: `reader`)

  - Uploaded file share role

- `~share_with_link` (`bool`, default: `true`)

  - Uploaded file share with link or not 

## Services

### `gdrive_ros/Upload`

This service is for uploading single file in same Google Drive folder.

**Request**

- `file_path` (`string`, default: `''`)

  - Uploaded file path

- `file_title` (`string`: `file_path.split('/')[-1]`)

  - Uploaded file title in Google Drive


- `parents_path` (`string`, default: `''`)

  - Parents path in Google Drive splitted by `/`


- `parents_id` (`string`, default: `''`)

  - Parents id in Google Drive

  - If both `parents_path` and `parents_id` are set , `parents_id` will be used.

- `use_timestamp_folder` (`bool`, default: `false`)

  - Use timestamp folder to upload

  - Uploaded file will be saved in `file_path/timestamp` folder.

- `use_timestamp_file_title` (`bool`, default: `false`)

  - Use timestamp for `file_title`

  - Uploaded file will be save as `{}_{}.format(timestamp file_title)`. 


**Response**

- `success` (`bool`)

  - Upload succeeded or not

- `file_id` (`string`)

  - Uploaded file id in Google Drive

- `file_url` (`string`)

  - Uploaded file url in Google Drive

- `parents_id` (`string`)

  - Parents folder id of uploaded file in Google Drive

- `parents_url` (`string`)

  - Parents folder url of uploaded file in Google Drive

### `gdrive_ros/MultipleUpload`

This service is for uploading multiple files in same Google Drive folder.

**Request**

- `file_paths` (`string[]`, default: `[]`)

  - Uploaded file paths

- `file_titles` (`string`: `[f for f in file_paths.split('/')[-1]]`)

  - Uploaded file titles in Google Drive


- `parents_path` (`string`, default: `''`)

  - Parents path in Google Drive splitted by `/`


- `parents_id` (`string`, default: `''`)

  - Parents id in Google Drive

  - If both `parents_path` and `parents_id` are set , `parents_id` will be used.

- `use_timestamp_folder` (`bool`, default: `false`)

  - Use timestamp folder to upload

  - Uploaded file will be saved in `file_path/timestamp` folder.

- `use_timestamp_file_title` (`bool`, default: `false`)

  - Use timestamp for `file_title`

  - Uploaded file will be save as `{}_{}.format(timestamp file_title)`. 


**Response**

- `successes` (`bool[]`)

  - Upload succeeded or not

- `file_ids` (`string[]`)

  - Uploaded file ids in Google Drive

- `file_urls` (`string[]`)

  - Uploaded file urls in Google Drive

- `parents_id` (`string`)

  - Parents folder id of uploaded file in Google Drive

- `parents_url` (`string`)

  - Parents folder url of uploaded file in Google Drive
