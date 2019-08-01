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

# --------------------------------------------------------------
# Derived class of filecmp.dircmp for recursive comparison
# --------------------------------------------------------------


class Compare(filecmp.dircmp):

    def phase3(self):
        """
        Override for recursive comparison
        :return: None
        """
        fcomp = filecmp.cmpfiles(self.left, self.right, self.common_files, shallow=False)
        self.same_files, self.diff_files, self.funny_files = fcomp
        return


def ldiffer(file1, file2):
    """
    Print comparison of two files
    :param file1: Name of the file 1
    :param file2: Name of the file 2
    :return: None
    """
    print(" ----- Compare files : %s and %s ------------- " % (file1, file2))

    with open(file1, 'r') as f1, open(file2, 'r') as f2:
        differ = Differ()

        for line in difflib.unified_diff(f1.readlines(), f2.readlines()):
            print(line)

    print(" ---- End compare ----------------------------")


# ----------------------------------------------------------------
# Method to identify ignored files
# ----------------------------------------------------------------

def valuable_item(item):
    """
    Identify files or directory to be ignore in the data archive
    :param item: File name or directory
    :return: None
    """

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

# -------------------------------------------------------------------
# Data packager object
# -------------------------------------------------------------------


class Packager(object):

    def __init__(self):
        """
        Initialize packager object.
        By default the package is connected to frydom-ce-data
        """
        self._file_archive = None
        self._version = tuple()
        self._additional_elements = []
        self._removed_data = []
        self._new_dir = []
        self._omitted_elements = []
        self._diff_files = []
        self._bucket = self._connect("frydom-ce-data")

    @property
    def str_version(self):
        """
        Return the current package version with format "x.y.z"
        :return: "x.y.z"
        """
        return '{}.{}.{}'.format(*self._version)

    def _connect(self, name_bucket):
        """
        Method to connect to a specific bucket in Amazon S3
        :param name_bucket: Name of the bucket
        :return: Bucket object
        """
        print("Connect resources S3 : %s" % name_bucket)
        s3 = boto3.resource('s3')
        return s3.Bucket(name_bucket)

    def get_last_data(self, nhead=0):
        """
        Find the data archive with the highest version number.
        If nhead not equal to 0, find the package at the nth position before the last package
        Save the package name on file_archive
        :param nhead: Position of the file archive before the file with the highest version number
        :return: None
        """
        files = list(self._bucket.objects.filter(Prefix="demo/data_"))
        self._file_archive, str_version = self.find_version(files, nhead)

        print("Last remote version : %s" % str_version)
        return

    def download_file_archive(self, version="", nhead=0):
        """
        Download a specific data archive from Amazon S3
        :param version: Version number with format 'x.y.z' (optional)
        :param nhead: Position of the file archive before the file with the highest version number (optional)
        :return: None
        """
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
        return

    def extract_file_archive(self):
        """
        Extract file archive to a temporary folder .temp/
        :return: None
        """
        with tarfile.open(self._file_archive, 'r') as f:
            f.extractall(".temp/")
        return

    def add(self, fname):
        """
        Add folder or file to the archive
        :param fname: Name of the new item
        :return: None
        """
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

    def remove(self, fname):
        """
        Remove folder or file to the archive
        :param fname: Name of the item to be removed
        :return: None
        """
        self._removed_data.append(fname)
        return

    def find_version(self, files, nhead=0):
        """
        Find the version number of the file placed at the nth position before
        the file with the highest version number on Amazon S3
        :param files: List of files
        :param nhead: Nth position before head
        :return:
            file_archive: Name of the file archive
            str_version: Version number with format 'x.y.z'
        """
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

    def _compare_folders(self, dir1, dir2):
        """
        Compare files of two folders
        :param dir1: First folder (left)
        :param dir2: Second folder (right)
        :return: None
        """
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

    def compare_to_local(self):
        """
        Compare files of the archive with local files
        :return: None
        """
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

        if len(self._removed_data) > 0:
            print("\n--> Removed files : \n")
            for elem in self._removed_data:
                print("    %s" % elem)

        print("")
        return
        
    def update_file(self, fname):
        """
        Add file to the updated list
        :param fname: Name of the file
        :return: None
        """
        if fname not in self.updated_files:
            print("Adding %s to archive" % fname)
            self.updated_files.append(fname)
        else:
            print("warning : %s already in archive (not add).")
        return
            
    def update_version(self, version):
        """
        Update the version number of the archive
        :param version: Type of revision (release/major/minor) or version number (x.y.z)
        :return: None
        """
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

    def update_package_version(self):
        """
        Update the package version information file
        :return: None
        """
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
            return

    def update(self, version):
        """
        Update archive
        :param version: Type of revision (release/major/minor) or version (x.y.z)
        :return: None
        """
        self.compare_to_local()

        if len(self._additional_elements) > 0 or len(self._removed_data) > 0:
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

            for relpath in self._removed_data:
                path = os.path.join('.temp', relpath)
                if os.path.isdir(path):
                    rmtree(path)
                elif os.path.isfile(path):
                    os.remove(path)

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

            print("***************************************************")
            print("* New archive is created successfully")
            print("* File : %s" % self._file_archive)
            print("* Version : %s" % self.str_version)
            print("* ")
            print("* Not follow to update your project with corresponding version : ")
            print("*     - ResourcePath.conf.cmake ")
            print("*     - URL_DEMO_DATA.conf.cmake ")
            print("****************************************************")

        else:
            print("Archive %s is already up to date" % self._file_archive)

    def upload(self):
        """
        Upload archive on Amazon S3
        :return: None
        """
        print("-----------------------------------------")
        print("Upload %s to AWS S3" % self._file_archive)
        print("-----------------------------------------")
        with open(self._file_archive, 'rb') as data:
            self._bucket.upload_fileobj(data, "demo/"+self._file_archive, ExtraArgs={'ACL':'public-read'})
        return





