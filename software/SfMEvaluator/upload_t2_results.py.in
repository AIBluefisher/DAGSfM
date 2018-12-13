# ----------------------------------------------------------------------------
# -                   TanksAndTemples Website Toolbox                        -
# -                    http://www.tanksandtemples.org                        -
# ----------------------------------------------------------------------------
# The MIT License (MIT)
#
# Copyright (c) 2017
# Arno Knapitsch <arno.knapitsch@gmail.com >
# Jaesik Park <syncle@gmail.com>
# Qian-Yi Zhou <Qianyi.Zhou@gmail.com>
# Vladlen Koltun <vkoltun@gmail.com>
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
# ----------------------------------------------------------------------------
#
# This python script is for uploading results to www.tanksandtemples.org
#
# Requirements:
# 	sudo pip install requests (requests is not included in python 2.7)

import json, os, sys
import datetime
import base64
import time
import sys
import os
import argparse
import zipfile
import hashlib
import requests

if (sys.version_info > (3, 0)):
	pversion = 3
	from urllib.request import Request, urlopen
else:
	pversion = 2
	from urllib2 import urlopen


sep = os.sep
parser = argparse.ArgumentParser(description='Tanks and Temples file uploader')

# Optional arguments
parser.add_argument('--group', type=str, help='(intermediate|advanced|both) choose if you want to download intermediate or advanced dataset', default='both')
parser.add_argument('--pathname', type=str, help='chose destination path name, default = local path', default='')
parser.add_argument('--send_md5_off', action='store_false', default=True, dest='send_md5', help='do not calculate md5sum after download')

GCS_API_ENDPOINT = 'https://storage.googleapis.com'


def Base64Sign(plaintext, key):
    """Signs and returns a base64-encoded SHA256 digest."""
    shahash = SHA256.new(plaintext)
    signer = PKCS1_v1_5.new(key)
    signature_bytes = signer.sign(shahash)
    return base64.b64encode(signature_bytes)


def submit_file(filename, credentials_upload):
	with open(credentials_upload) as f:
		content = f.readlines()
	credential_content = [x.strip() for x in content]

	signature_dict = {}
	policy_dict = {}
	for signatureline in credential_content[0:-1]:
		signaturelinecontent = signatureline.split('###')
		signature_dict[signaturelinecontent[2]]=signaturelinecontent[1]
		policy_dict[signaturelinecontent[2]]=signaturelinecontent[0]

	access_str = credential_content[-1]
	access_info = access_str.split('###')
	fsignature = signature_dict[filename]
	fpolicy = policy_dict[filename]
	gcs_filename = access_info[1] + filename
	client_id_email = access_info[2]
	expiration = access_info[3]

	files = {'file': open(filename,'rb')}
	gs_acl = 'bucket-owner-read'
	headers = {'enctype': 'multipart/form-data'}
	bucket_name = 't2-website-userdata'
	policy = {}
	policy['key'] = gcs_filename
	policy['bucket'] = 't2-website-userdata'
	policy['acl'] = gs_acl

	policy['GoogleAccessId'] = client_id_email
	policy['bucket'] = bucket_name
	policy['policy'] = fpolicy
	policy['signature'] = fsignature
	url = 'http://{bucket_name}.storage.googleapis.com'.format(bucket_name=bucket_name)
	session = requests.Session()
	print('start %s upload' % filename)
	r = session.post(url, data=policy, files=files, headers=headers)
	ProcessResponse(r,204)


def ProcessResponse(r, expected_status=200):
  if r.status_code != expected_status:
    sys.exit('Exiting due to receiving %d status code when expecting %d.'
             % (r.status_code, expected_status))
  else:
	  print('upload finished')


def generate_file_md5(filename, blocksize=2**20):
    m = hashlib.md5()
    with open(filename, "rb") as f:
        while True:
            buf = f.read(blocksize)
            if not buf:
                break
            m.update( buf )
    return m.digest()


def generate_file_md5X(filename, blocksize=2**20):
    m = hashlib.md5()
    with open(filename, "r") as f:
        while True:
            buf = f.read(blocksize)
            if not buf:
                break
            m.update( buf )
    return m.digest()


def generate_md5_file(md5_check_fn, scene_list):
	md5_check = open(md5_check_fn, 'wb')
	for scene in scene_list:
		ply_file = scene + '.ply'
		log_file = scene + '.log'
		if os.path.isfile(ply_file):
			md5_ply_file = generate_file_md5(ply_file, blocksize=2**20)
		else:
			md5_ply_file = ''
		if os.path.isfile(log_file):
			md5_log_file = generate_file_md5(log_file, blocksize=2**20)
		else:
			md5_log_file = ''
		content_md5_log = base64.b64encode(md5_log_file)
		content_md5_ply = base64.b64encode(md5_ply_file)
		print('md5_ply_file: ', ply_file, content_md5_ply)
		print('md5_log_file: ',log_file, content_md5_log)
		md5_check.write("%s###%s\n" % (ply_file, content_md5_ply.decode('UTF-8')))
		md5_check.write("%s###%s\n" % (log_file, content_md5_log.decode('UTF-8')))
	md5_check.close()


def check_filestatus(scene_list):
	for scene in scene_list:
		ply_file = scene + '.ply'
		log_file = scene + '.log'
		if not os.path.isfile(ply_file):
			sys.exit('Error! %s is missing.' % ply_file)
		if not os.path.isfile(log_file):
			sys.exit('Error! %s is missing.' % log_file)

intermediate_list = ['Family','Francis','Horse','Lighthouse','M60','Panther','Playground','Train']
advanced_list = ['Auditorium','Ballroom','Courtroom','Museum','Palace','Temple']
both_list = intermediate_list + advanced_list
both_list.sort()

args = parser.parse_args()
sequences = args.group
send_md5 = args.send_md5

if sequences == 'intermediate':
	scene_list = intermediate_list
elif sequences == 'advanced':
	scene_list = advanced_list
elif sequences == 'both':
	scene_list = intermediate_list + advanced_list
elif sequences == '':
	scene_list = intermediate_list + advanced_list
else:
	sys.exit('Error! Unknown group parameter, see help [-h]')

scene_list.sort()

check_filestatus(scene_list)

credentials_upload = 't2_submission_credentials.txt'
md5_check_fn = 'md5_check.txt'
generate_md5_file(md5_check_fn, both_list)

if send_md5:
	submit_file(md5_check_fn, credentials_upload)

for scene in scene_list:
	ply_file = scene + '.ply'
	log_file = scene + '.log'
	submit_file(ply_file, credentials_upload)
	submit_file(log_file, credentials_upload)
