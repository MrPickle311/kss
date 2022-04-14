#!/usr/bin/env python3

from http_io.http_sender import HttpSender
import base64
import os

from data_models.kss_clients import JsonImageFormat, JsonImagesMessageFormat
from typing import List
from flask import Response


class ImageSender(HttpSender):
    MAX_IMAGES_COUNT_IN_MSG = 50

    def __init__(self, host_address: str, port: int, station_id: int, images_directory: str):
        HttpSender.__init__(self, host_address, port, 'image', station_id, 'api')
        self._images_directory = images_directory

    @staticmethod
    def _check_is_file(image_dir: str) -> None:
        if not os.path.isfile(image_dir):
            raise TypeError('An image is not a path file dir!')

    @staticmethod
    def _convert_image_to_base64(image: str) -> str:

        with open(image, 'rb') as binary_image:
            image_content = base64.b64encode(binary_image.read()).decode('utf-8')

        return image_content

    @staticmethod
    def _get_file_name(file: str) -> str:
        return os.path.basename(file)

    @staticmethod
    def _extract_json_image(image_path: str) -> JsonImageFormat:
        image_json = JsonImageFormat(name=ImageSender._get_file_name(image_path),
                                     image=ImageSender._convert_image_to_base64(image_path))
        return image_json

    def grab_all_images_names(self, directory: bytes) -> List[str]:
        image_dirs = []
        for image_file in os.listdir(directory):
            if image_file.endswith(b'.jpg'):
                image_dirs.append(self._images_directory + image_file.decode('utf-8'))
        return image_dirs

    def get_splitted_images_path_lists(self, image_paths: List[str]) -> List[List[str]]:
        return [image_paths[i:i + self.MAX_IMAGES_COUNT_IN_MSG] for i in
                range(0, len(image_paths), self.MAX_IMAGES_COUNT_IN_MSG)]

    def _create_msg(self, images_paths_to_send: List[str], region_id: int) -> JsonImagesMessageFormat:
        data = JsonImagesMessageFormat()

        data.station_id = self.station_id
        data.region_id = region_id

        data.files.clear()

        for image_dir in images_paths_to_send:
            data.files.append(ImageSender._extract_json_image(image_dir))

        return data

    def clear_directory(self):
        for file in os.scandir(self._images_directory):
            os.remove(file.path)

    @staticmethod
    def remove_files(files: List[str]) -> None:
        map(os.remove, files)

    def send_images(self, region_id: int) -> None:
        directory = os.fsencode(self._images_directory)
        images_paths = self.grab_all_images_names(directory)
        images_paths_sublists = self.get_splitted_images_path_lists(images_paths)

        for images_sublist in images_paths_sublists:
            data = self._create_msg(images_sublist, region_id)
            resp: Response = self.send_message(data)
            if resp.status_code != 200:
                break
            self.remove_files(images_sublist)

        # self.clear_directory()
