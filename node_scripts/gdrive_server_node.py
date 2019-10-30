#!/usr/bin/env python

import datetime
import os.path
import sys

from pydrive.auth import GoogleAuth
from pydrive.drive import GoogleDrive
from pydrive.files import ApiRequestError
import rospy

from gdrive_ros.srv import Upload
from gdrive_ros.srv import UploadResponse


class GDriveServerNode(object):
    folder_mime_type = 'application/vnd.google-apps.folder'
    folder_url_format = 'https://drive.google.com/drive/folders/{}'
    file_url_format = 'https://drive.google.com/uc?id={}'

    def __init__(self):
        settings_yaml = rospy.get_param('~settings_yaml', None)
        if settings_yaml is not None:
            self.gauth = GoogleAuth(settings_yaml)
        else:
            rospy.logerr('param: ~settings_yaml is not correctly set.')
            sys.exit(1)

        rospy.loginfo('Google drive authentication starts.')
        self.gauth.LocalWebserverAuth()
        self.gdrive = GoogleDrive(self.gauth)
        rospy.loginfo('Google drive authentication finished.')
        self.upload_server = rospy.Service('~upload', Upload, self._upload_cb)

    def _upload_cb(self, req):
        timestamp = '{0:%Y%m%d%H%M%S}'.format(datetime.datetime.now())
        parents_path = req.parents_path
        parents_id = req.parents_id
        file_title = req.file_title
        file_path = os.path.expanduser(req.file_path)
        res = UploadResponse()
        res.success = False

        if parents_id and parents_path:
            rospy.logerr('parents_path and parents_id is both set.')
            rospy.logerr('parents_id: {} is selected to upload.')
            parents_path = ''

        if parents_path:
            try:
                parents_id = self._get_parents_id(
                    parents_path, mkdir=True)
            except (ValueError, ApiRequestError) as e:
                rospy.logerr(e)
                rospy.logerr(
                    'Failed to get parents_id: {}'.format(parents_path))
                return res
        # root
        elif parents_id == '' and parents_path == '':
            parents_id = ''

        if req.use_timestamp_folder:
            try:
                parents_id = self._get_parents_id(
                    [timestamp], parents_id=parents_id, mkdir=True)
            except (ValueError, ApiRequestError) as e:
                rospy.logerr(e)
                rospy.logerr(
                    'Failed to get parents_id: {} in {}'.format(
                        timestamp, self.folder_url_format.format(parents_id)))
                return res

        file_title = file_title if file_title else file_path.split('/')[-1]
        if req.use_timestamp_file_title:
            file_title = '{}_{}'.format(timestamp, file_title)

        try:
            file_id = self._upload_file(
                file_path, file_title, parents_id=parents_id)
            res.success = True
            res.file_id = file_id
            res.file_url = self.file_url_format.format(file_id)
            res.parents_id = parents_id
            res.parents_url = self.folder_url_format.format(parents_id)
        except ApiRequestError as e:
            rospy.logerr(e)
            rospy.logerr('Failed to upload: {} -> {}',
                         file_path, self.folder_url_format.format(parents_id))
        return res

    def _upload_file(self, file_path, file_title, parents_id=None):
        rospy.loginfo('Start uploading a file: {}'.format(file_title))
        if parents_id:
            gfile = self.gdrive.CreateFile(
                {'parents': [{'id': parents_id}]})
        else:
            gfile = self.gdrive.CreateFile()
        gfile.SetContentFile(file_path)
        gfile['title'] = file_title
        gfile.Upload()
        rospy.loginfo('Finish uploading a file: {}'.format(file_title))
        return gfile['id']

    def _upload_folder(self, folder_title, parents_id=None):
        rospy.loginfo('Start making a folder: {}'.format(folder_title))
        if parents_id:
            gfolder = self.gdrive.CreateFile(
                {'title': folder_title,
                 'parents': [{'id': parents_id}],
                 'mimeType': 'application/vnd.google-apps.folder'})
        else:
            gfolder = self.gdrive.CreateFile(
                {'title': folder_title,
                 'mimeType': 'application/vnd.google-apps.folder'})
        gfolder.Upload()
        rospy.loginfo('Finish making a folder: {}'.format(folder_title))
        return gfolder['id']

    def _get_parents_id(self, parents_path, parents_id=None, mkdir=False):
        if parents_path == '':
            return None

        if not isinstance(parents_path, list):
            parents_path = [p for p in parents_path.split('/') if p != '']

        folder_title = parents_path[0]
        parent = parents_id if parents_id else 'root'
        gfiles = self.gdrive.ListFile(
            {'q': "'{}' in parents and trashed=false".format(parent)})
        gfiles = gfiles.GetList()
        gfolders = []
        for gf in gfiles:
            if (gf['mimeType'] == self.folder_mime_type
                    and gf['title'] == folder_title):
                gfolders.append(gf)

        if len(parents_path) == 1:
            if len(gfolders) > 0:
                return gfolders[0]['id']
            if mkdir:
                folder_id = self._upload_folder(
                    folder_title, parents_id=parents_id)
                return folder_id
            else:
                raise ValueError(
                    'Folder is not found: {}'.format(folder_title))
        else:
            if len(gfolders) > 0 or mkdir:
                if len(gfolders) > 0:
                    next_parents_id = gfolders[0]['id']
                elif mkdir:
                    next_parents_id = self._upload_folder(
                        folder_title, parents_id=parents_id)
                folder_id = self._get_parents_id(
                    parents_path[1:], parents_id=next_parents_id,
                    mkdir=mkdir)
                return folder_id
            else:
                raise ValueError('folder is not found: {}', folder_title)


if __name__ == '__main__':
    rospy.init_node('gdrive_server')
    node = GDriveServerNode()
    rospy.spin()
