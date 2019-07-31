import boto3
import os, sys
import tarfile
import filecmp
from difflib import Differ
import difflib
from shutil import copyfile, rmtree
import numpy as np
from datetime import date, datetime
import time
import re


class Packager(object):

    def __init__(self):
        self._file_archive = None
        self._version = tuple()
        self._additional_elements = []
        self._new_dir = []
        self._omitted_elements = []
        self._diff_files = []
        self._bucket = self._connect("frydom-ce-data")

    @property
    def str_version(self):
        return '{}.{}.{}'.format(*self._version)

    def _connect(self, name_bucket):
        print("Connect resources S3 : %s" % name_bucket)
        s3 = boto3.resource('s3')
        return s3.Bucket(name_bucket)

    def get_last_data(self, nhead=0):
        files = list(self._bucket.objects.filter(Prefix="demo/data_"))
        self._file_archive, str_version = self.find_version(files, nhead)

        print("Last remote version : %s" % str_version)

    def download_file_archive(self, version="", nhead=0):

        if not version and nhead == 0:
            self.get_last_data()
        elif not version:
            self.get_last_data(nhead)
        elif version:
            self._version = list(int(val) for val in version.split('.'))
            self._file_archive = "data_v" + version + ".tar.gz"

        if not os.path.isfile(self._file_archive):
            print("Downloading : %s" % self._file_archive)
            self._bucket.download_file('demo/' + self._file_archive, self._file_archive)
        else:
            pass

    def extract_file_archive(self):
        with tarfile.open(self._file_archive, 'r') as f:
            f.extractall(".temp/")

    def add(self, fname):

        if os.path.isdir(fname):
            for (dirpath, dirnames, filenames) in os.walk(fname):
                for file in filenames:
                    filepath = os.path.join(dirpath, file)
                    if filepath not in self._additional_elements:
                        self._additional_elements.append(os.path.normpath(filepath))
        elif os.path.isfile(fname):
            self._additional_elements.append(os.path.normpath(fname))
        else:
            print("warning : %s unknown file or dir" % fname)

        return

    def _compare_folders(self, dir1, dir2):

        comp = Compare(dir1, dir2)

        new_dir = [os.path.normpath(os.path.join(dir1, item)) for item in comp.left_only if valuable_item(item)]

        for elem in new_dir:
            if os.path.isdir(elem):
                for (dirpath, dirnames, filenames) in os.walk(elem):
                    for file in filenames:
                        self._new_dir.append(os.path.join(dirpath, file))
            elif os.path.isfile(elem):
                self._new_dir.append(os.path.normpath(elem))

        diff_files = [os.path.normpath(os.path.join(dir1, item)) for item in comp.diff_files]

        self._diff_files.extend(diff_files)

        for name in comp.diff_files:
            ldiffer(os.path.join(dir1, name), os.path.join(dir2, name))

        for subdir in comp.common_dirs:
            self._compare_folders(os.path.join(dir1, subdir), os.path.join(dir2, subdir))

        return

    def find_last_version(self, files):

        list_version = []
        for file in files:
            file = file.key[5:]
            list_version.append(list(int(val) for val in file[6:-7].split('.')))

        temp = [val for val in list_version if val[0] == max(list_version)[0]]
        temp2 = [val for val in temp if val[1] == max(temp)[1]]
        temp3 = [val for val in temp2 if val[2] == max(temp2)[2]]

        if len(temp3) > 1 or len(temp3) == 0:
            print("error : unable to find maximum version")
        else:
            self._version = temp3[0]

        str_version = '{}.{}.{}'.format(*self._version)
        file_archive = "data_v" + str_version + ".tar.gz"

        return file_archive, str_version

    def find_version(self, files, nhead=0):

        table_version = np.array([], dtype=np.uint8)
        for file in files:
            file = file.key[5:]
            list_version = list(int(val) for val in file[6:-7].split('.'))
            table_version = np.append(table_version, list_version)
        table_version = table_version.reshape((len(files), 3))

        table_ordered = table_version[np.lexsort(np.fliplr(table_version).T)]

        self._version = table_ordered[-nhead-1]

        str_version = '{}.{}.{}'.format(*self._version)
        file_archive = "data_v" + str_version + ".tar.gz"

        return file_archive, str_version

    def compare_to_local(self):

        print("Check for update... ")

        self.extract_file_archive()

        self._compare_folders('.', '.temp/')

        temp = []
        sys.stdout.write("\033[1;31m")
        for elem in self._additional_elements:
            if elem in self._new_dir:
                temp.append(elem)
            else:
                print("update info: %s is already present in archive: omitted to completion" % elem)
        self._additional_elements = temp

        sys.stdout.write("\033[0;0m")

        for item in self._new_dir:
            if item not in self._additional_elements:
                self._omitted_elements.append(item)

        if len(self._additional_elements) > 0:
            print("\n--> Added to archive :\n")
            sys.stdout.write("\033[0;32m")
            for elem in self._additional_elements:
                print("    %s" % elem)
            sys.stdout.write("\033[0;0m")

        if len(self._omitted_elements) > 0:
            print("\n--> Omitted elements : \n")
            sys.stdout.write("\033[1;31m")
            for elem in self._omitted_elements:
                print("    %s" % elem)
            sys.stdout.write("\033[0;0m")

        if len(self._diff_files) > 0:
            print("\n--> Diff files : \n")
            for elem in self._diff_files:
                print("    %s" % elem)

        print("")
        
    def update_file(self, fname):
        if fname not in self.updated_files:
            print("Adding %s to archive" % fname)
            self.updated_files.append(fname)
        else:
            print("warning : %s already in archive (not add).")
        return
            
    def update_version(self, version):
        
        if version == "release":
            self._version[0] += 1
            self._version[1] = 0
            self._version[2] = 0
        elif version == "major":
            self._version[1] += 1
            self._version[2] = 0
        elif version == "minor":
            self._version[2] += 1
        else:
            try:
                self._version = list(int(val) for val in version.split('.'))
            except:
                print("warning : unknown type version.")
            
        return

    def update(self, version):

        self.compare_to_local()

        if len(self._additional_elements) > 0:
            print("Updating archive...")

            self.update_version(version)
            str_version = '{}.{}.{}'.format(*self._version)

            new_archive = "data_v" + str_version + ".tar.gz"
            copyfile(self._file_archive, new_archive)

            if os.path.exists('.temp/'):
                rmtree('.temp/')
            with tarfile.open(self._file_archive, 'r') as f:
                print("--> Extract data from previous archive")
                f.extractall(".temp/")

            self.update_package_version()

            with tarfile.open(new_archive, 'w:gz') as f:
                print("--> Create new archive")
                for item in os.listdir('.temp/'):
                    f.add(item)
                for item in self._additional_elements:
                    f.add(item, arcname=item)

                f.add("package_version.info")

            self._file_archive = new_archive

            print("--> Delete temporary data")
            rmtree('.temp/')
            print("New archive %s created" % new_archive)
        else:
            print("Archive %s is already up to date" % self._file_archive)

    def upload(self):
        print("Upload %s to AWS S3" % self._file_archive)
        with open(self._file_archive, 'rb') as data:
            self._bucket.upload_fileobj(data, "demo/"+self._file_archive, ExtraArgs={'ACL':'public-read'})

    def update_package_version(self):

        with open(os.path.join("package_version.info"), 'w') as f:

            d = datetime.now()

            f.write("Version : %s\n" % self.str_version)
            f.write("Date : %s\n" % d.strftime("%Y/%m/%d %H:%M"))
            f.write("\n")

            for (dirpath, dirnames, filenames) in os.walk('.temp/'):
                for file in filenames:
                    f.write(os.path.join(os.path.basename(dirpath), file)+"\n")

            for file in self._additional_elements:
                f.write(file+"\n")


def print_diff_files(dcmp):
    for name in dcmp.diff_files:
        print("diff_file %s found in %s and %s" % (name, dcmp.left, dcmp.right))
    for sub_dcmp in dcmp.subdirs.values():
        print_diff_files(sub_dcmp)


class Compare(filecmp.dircmp):
    
    def phase3(self):
        fcomp = filecmp.cmpfiles(self.left, self.right, self.common_files, shallow=False)
        self.same_files, self.diff_files, self.funny_files = fcomp


def ldiffer(file1, file2):
    
    print(" ----- Compare files : %s and %s ------------- " % (file1, file2))
    
    with open(file1, 'r') as f1, open(file2, 'r') as f2:
        differ = Differ()
        
        for line in difflib.unified_diff(f1.readlines(), f2.readlines()):
            print(line)
            
    print(" ---- End compare ----------------------------")


def valuable_item(item):
       
    res = True   
       
    try:
        if item[-6:] == 'tar.gz':
            return False
    except:
        pass

    try:
        if item == 'DataPackager.py':
            return False
    except:
        pass
    
    try:
        if item[0] == '.':
            return False
    except:
        pass
        
        
    return True
    
        
def compare_directories(dir1, dir2):

    comp = Compare(dir1, dir2)

    new_dir = [item for item in comp.left_only if valuable_item(item)]

    format_list = [dir1+'/{:>3}' for item in new_dir]
    sl = '\n'.join(format_list)+'\n'
    list_left_only = sl.format(*new_dir)

    format_list = [dir1+'/{:>3}' for item in comp.diff_files]
    sd = '\n'.join(format_list)+'\n'
    list_diff_files = sd.format(*comp.diff_files)

    for name in comp.diff_files:
        ldiffer(os.path.join(dir1, name), os.path.join(dir2, name))

    for subdir in comp.common_dirs:
        res1, res2 = compare_directories(os.path.join(dir1, subdir), os.path.join(dir2, subdir))
        list_left_only += res1
        list_diff_files += res2

    return list_left_only, list_diff_files


def report_compared(text1, text2):
    
    print("")
    
    print(" -- New folder :\n")
    print(text1)
    
    print(" -- Diff files : \n")
    print(text2)
    

def main():

    data = Packager()

    data.download_file_archive()

    data.add2("./FOSWEC")
    data.add2("./Cylinder")
    data.add2("./bench/sphere")

    data.update("minor")

    #data.upload()


if __name__ == "__main__":
    main()





