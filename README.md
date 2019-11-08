# gdrive_ros
[![GitHub version](https://badge.fury.io/gh/knorth55%2Fgdrive_ros.svg)](https://badge.fury.io/gh/knorth55%2Fgdrive_ros)
[![Build Status](https://travis-ci.com/knorth55/gdrive_ros.svg?branch=master)](https://travis-ci.com/knorth55/gdrive_ros)

Google Drive file uploader for ROS

## Installation

### Setup and build workspace 

```bash
cd ~
mkdir gdrive_ws/src -p
cd gdrive_ws/src
git clone https://github.com/knorth55/gdrive_ros.git
rosdep install --ignore-src -from-paths . -y -r -i
cd ~/gdrive_ws
catkin build
```

### Authentication for Google Drive API 

Please follow [here](https://gsuitedevs.github.io/PyDrive/docs/build/html/quickstart.html#authentication).

### Create your settings yaml

Please follow [here](https://gsuitedevs.github.io/PyDrive/docs/build/html/oauth.html#sample-settings-yaml).

My settings yaml is as follows;

```yaml
client_config_file: /your/client/secrets/json/path
save_credentials: True
save_credentials_backend: file
save_credentials_file: /your/credentials/json/path
get_refresh_token: True
oauth_scope:
  - https://www.googleapis.com/auth/drive.file
```

## Usage

### Run server
```bash
export GOOGLE_DRIVE_SETTINGS_YAML=/your/settings/yaml/path
roslaunch gdrive_ros gdrive_server.launch
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
