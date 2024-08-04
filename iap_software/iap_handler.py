from serial import Serial
import struct
import time
import os
import sys

class iap_handler:
	PROGRESS_LINE = "=" * 50
	MAX_DATA_SZ = 256
	IAP_FORMAT = "=HH%uBI"%(MAX_DATA_SZ)
	IAP_OP = {
		"INTERACT" : 0,
		"LAUNCH_APP" : 1,
		"UPLOAD_APP" : 2,
		"UPLOADING" : 3,
		"UPLOAD_COMPLETE" : 4,
		"ABORT_UPLOAD" : 5,
		"UPLOAD_ERR" : 6,
		"UPDATE_FLASH" : 7,
		"UPDATE_FLASH_RET" : 8,
		"UPDATE_FLASH_FINISH" : 9,
		"ERR" : 0x20,
	}

	IAP_LOG_PREFIX = "[IAP]"

	def __init__(self, port="/dev/ttyUSB0", baudrate=115200, timeout=None):
		self.iap_serial = Serial(port=port, baudrate=baudrate, timeout=timeout)

	def send(self, op, data=[]):
		data_len = len(data)
		if type(data) == bytes:
			data = [int(d) for d in data]
		data.extend([0] * (self.MAX_DATA_SZ - len(data)))
		checksum = sum(struct.pack(self.IAP_FORMAT, op, data_len, *data, 0))
		send_data = struct.pack(self.IAP_FORMAT, op, data_len, *data, checksum)
		self.iap_serial.write(send_data)

	def recv(self, dump_flag=False):
		IAP_FORMAT_SZ = struct.calcsize(self.IAP_FORMAT)
		recv_data = self.iap_serial.read(IAP_FORMAT_SZ)
		recv_data = struct.unpack(self.IAP_FORMAT, recv_data)
		if dump_flag:
			self.__dump_data(recv_data)
		recv_data = {
			"op": recv_data[0],
			"len": recv_data[1],
			"data": recv_data[2:(2 + recv_data[1])],
			"checksum": recv_data[-1]
		}
		return recv_data

	def interact(self, dump_flag=False) -> bool:
		self.send(self.IAP_OP["INTERACT"])
		ret_data = self.recv(dump_flag)
		return self.__match_op(ret_data["op"], self.IAP_OP["INTERACT"], "enter to the interaction mode!")


	def launch_app(self, dump_flag=False) -> bool:
		self.send(self.IAP_OP["LAUNCH_APP"])
		ret_data = self.recv(dump_flag)
		return self.__match_op(ret_data["op"], self.IAP_OP["LAUNCH_APP"], "launch the application!")

	def upload_bin(self, filename, upload_temp=False, dump_flag=False) -> bool:
		self.send(self.IAP_OP["UPLOAD_APP"])
		ret_data = self.recv(dump_flag)
		upload_ret_flag = self.__match_op(ret_data["op"], self.IAP_OP["UPLOAD_APP"], "enter to the upload mode!")
		time.sleep(1)
		with open(filename, "rb") as binary:
			binary.seek(0, os.SEEK_END)
			bin_size = binary.tell()
			bin_sent_size = 0
			progress_line = self.PROGRESS_LINE[0: int(bin_sent_size / bin_size * len(self.PROGRESS_LINE))]
			self.__log("upload progress: [%-*s] %7.2f%%"%(len(self.PROGRESS_LINE), 
				   progress_line, bin_sent_size / bin_size * 100))
			binary.seek(0, os.SEEK_SET)
			read_bytes = [True]
			while read_bytes:
				read_bytes = binary.read(iap_handler.MAX_DATA_SZ)
				if (len(read_bytes) == 0):
					break;
				self.send(self.IAP_OP["UPLOADING"], read_bytes)
				ret_data = self.recv(dump_flag)
				while ret_data["op"] != self.IAP_OP["UPLOADING"]:
					print("send error:", ret_data["op"])
					self.send(self.IAP_OP["UPLOADING"], read_bytes)
					ret_data = self.recv(dump_flag)
					
				bin_sent_size += len(read_bytes)
				progress_line = self.PROGRESS_LINE[0: int(bin_sent_size / bin_size * len(self.PROGRESS_LINE))]
				self.__log("upload progress: [%-*s] %7.2f%%"%(len(self.PROGRESS_LINE), 
					    progress_line, bin_sent_size / bin_size * 100))
		
		if upload_temp:
			self.send(self.IAP_OP["ABORT_UPLOAD"])
			ret_data = self.recv(dump_flag)
			return self.__match_op(ret_data["op"], self.IAP_OP["ABORT_UPLOAD"],
					       "abort upload! but the uploaded binary code is still stored in the storage!")
		else:
			self.send(self.IAP_OP["UPLOAD_COMPLETE"])
			ret_data = self.recv(dump_flag)
			return self.__match_op(ret_data["op"], self.IAP_OP["UPLOAD_COMPLETE"], "upload finish!")
	
	def update_flash(self, dump_flag=False):
		iap_handler_obj.send(self.IAP_OP["UPDATE_FLASH"])
		ret_data = iap_handler_obj.recv(dump_flag)
		while ret_data["op"] != self.IAP_OP["UPDATE_FLASH_FINISH"]:
			ret_data = iap_handler_obj.recv(dump_flag)

			if ret_data["op"] == self.IAP_OP["UPDATE_FLASH_RET"]:
				progress = struct.unpack("<f", bytes(ret_data["data"]))[0]
				progress_line = self.PROGRESS_LINE[0: int(progress * len(self.PROGRESS_LINE))]
				self.__log("update progress: [%-*s] %7.2f%%"%(len(self.PROGRESS_LINE),
					   progress_line, progress * 100))
		self.__log("update finish!")

	def __dump_data(self, ret_data):
		self.__log("[DUMP]", "op, length = 0x%04x, %4d"%(ret_data[0], ret_data[1]))
		
		num_of_bytes_in_line = 16
		delimiter = " " * (num_of_bytes_in_line - 1) + "\n"
		for i in range(0, self.MAX_DATA_SZ, num_of_bytes_in_line):
			self.__log("[DUMP]",  end="")
			partial_data = ret_data[2 + i:2 + i+ num_of_bytes_in_line]
			for j in range(len(partial_data)):
				print(" 0x%02x"%(partial_data[j]), 
				      end=delimiter[j & (num_of_bytes_in_line - 1)])
		self.__log("[DUMP]", "received checksum: 0x%08x"%(ret_data[-1]))
		
		checksum = sum(struct.pack(self.IAP_FORMAT, *(ret_data))[0:-4])
		self.__log("[DUMP]", "calculated checksum: 0x%08x"%(checksum))

	def __match_op(self, op, expected, expected_str) -> bool:
		if op != expected:
			self.__log("[Error]", "error: get returned op[%d]"%(op), file=sys.stderr)
			return False
		self.__log(expected_str)
		return True

	def __log(self, *args, **kwargs):
		print(self.IAP_LOG_PREFIX, *args, **kwargs)

if __name__ == "__main__":
	if len(sys.argv) < 2:
		print("No given binary file")
		exit(1)

	flag = False
	iap_handler_obj = iap_handler()

	if iap_handler_obj.interact():
		iap_handler_obj.upload_bin(sys.argv[1], dump_flag=flag)
		iap_handler_obj.update_flash(flag)
		iap_handler_obj.launch_app();
