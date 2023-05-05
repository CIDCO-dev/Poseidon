import requests, base64, json
import sys,os,shutil



def encode_file(zipFilePath):
	encondedZip = ""
	with open(zipFilePath, "rb") as f:
		    encodedZip = base64.b64encode(f.read())
	return encodedZip.decode()

def assemble_json(key, job_type, base64Zip):
	return {"jobType":job_type, "apiKey":key, "fileData":base64Zip}

def send_request(host, target, data):
	return requests.post(host+target, json=data)

def zip_all(path):
	toZip = dict()
	
	# put all fileName with same timestamp in an array
	for f in os.listdir(path):
		timestamp = f[:17]
		if timestamp not in toZip.keys():
			bundle = []
			bundle.append(f) 
			toZip[timestamp] = bundle
		elif timestamp in toZip.keys():
			bundle = toZip[timestamp]
			bundle.append(f) 
			toZip[timestamp] = bundle
			
	# put file in array in same directory
	for timestamp, files in toZip.items():
		zipPath = os.path.join(path,timestamp)
		if len(files) != 5:
			sys.stderr.write("Missing one file, skipping : {}\n".format(timestamp))
		else:
			if os.path.exists(zipPath):
				try:
					os.rmdir(zipPath)
					print("Directory '% s' has been removed successfully" % zipPath)
				except OSError as error:
					print(error)
					print("Directory '% s' can not be removed" % zipPath)
			os.mkdir(zipPath)
			for f in files:
				os.rename(os.path.join(path,f), os.path.join(zipPath,f))
	
	# zip directories 	
	for d in os.listdir(path):
		if os.path.isdir(os.path.join(path,d)):
			print(d)
			shutil.make_archive(format="zip", base_name=os.path.join(path,d), root_dir=os.path.join(path,d))
			

def transfer_all(host, target, key, jobType, path):
	for f in os.listdir(path):
		if not f.endswith(".zip"):
			continue
		response = send_request(host, target, assemble_json(key, jobType, encode_file(os.path.join(path,f))))
		if response.status_code == 200 :
			# delete zip
			os.remove(os.path.join(path,f))
			# delete directory
			try:
				shutil.rmtree(os.path.join(path,f[:17]))
			except:
				print('Error deleting directory')
				
			if os.path.exists(os.path.join(path,f)):
				print("Could not delete file after sending it")
		else:
			print("Transfer failed", response.status_code)

#======== MAIN =========#

if len(sys.argv) != 6:
        sys.stderr.write("Usage: python3 csb_api_call.py host target key jobType path \n")
        sys.exit(1)

host = sys.argv[1]
target = sys.argv[2]
key = sys.argv[3]
jobType = sys.argv[4]
path = sys.argv[5]

if not os.path.exists(path):
	print("Wrong path, path does not exist")
	sys.exit(1)


if not os.path.isdir(path):

	print("Path must be a directory")
	sys.exit(1)
	
else:
	zip_all(path)
	transfer_all(host, target, key, jobType, path)
	

