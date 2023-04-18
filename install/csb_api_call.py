import requests, base64, json
import sys,os


def encodeFile(zipFilePath):
	encondedZip = ""
	with open(zipFilePath, "rb") as f:
		    encodedZip = base64.b64encode(f.read())
	return encodedZip.decode()

def assembleJson(key, job_type, base64Zip):
	return {"jobType":job_type, "apiKey":key, "fileData":base64Zip}

def sendRequest(host, target, data):
	return requests.post(host+target, json=data)

#======== MAIN =========#

if len(sys.argv) != 6:
        sys.stderr.write("usage: python3 csb_api_call.py host target key jobType path \n")
        sys.exit(1)

host = sys.argv[1]
target = sys.argv[2]
key = sys.argv[3]
jobType = sys.argv[4]
path = sys.argv[5]

if not os.path.exists(path):
	print("wrong path, path does not exist")
	sys.exit(1)

if os.path.isfile(path): 
	response = sendRequest(host, target, assembleJson(key, jobType, encodeFile(path)))
	if response.status_code == 200 :
		os.remove(path)
		if os.path.exists(path):
			print("could not delete file after sending it")
	else:
		print("not ok", response.status_code)

elif os.path.isdir(path):
	for f in os.listdir(path):
		response = sendRequest(host, target, assembleJson(key, jobType, encodeFile(os.path.join(path,f))))
		if response.status_code == 200 :
			os.remove(os.path.join(path,f))
			if os.path.exists(os.path.join(path,f)):
				print("could not delete file after sending it")
		else:
			print("not ok", response.status_code)

