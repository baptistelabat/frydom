import boto3
import os, sys
import tarfile
import filecmp
from difflib import Differ
import difflib
from shutil import copyfile, rmtree


class DataBase(object):

    def __init__(self):
        self._file_archive = None
        self._version = tuple()
        self._additional_elements = []
        self._new_dir = []
        self._omitted_elements = []
        self._diff_files = []
        self._bucket = self._connect("frydom-ce-data")

    def _connect(self, name_bucket):
        print("Connect resources S3 : %s" % name_bucket)
        s3 = boto3.resource('s3')
        return s3.Bucket(name_bucket)

    def get_last_data(self):
        files = list(self._bucket.objects.filter(Prefix="demo/data_"))
        self._file_archive = files[-1].key[5:]

        print("Last remote version : %s" % self._file_archive[6:-7])
        self._version = list(int(val) for val in self._file_archive[6:-7].split('.'))

    def download_file_archive(self):

        self.get_last_data()

        if not os.path.isfile(self._file_archive):
            print("Downloading : %s" % self._file_archive)
            self._bucket.download_file('demo/' + self._file_archive, self._file_archive)
        else:
            pass

    def extract_file_archive(self):
        with tarfile.open(self._file_archive, 'r') as f:
            f.extractall(".temp/")

    def add(self, fname):
        if fname not in self._additional_elements:
            self._additional_elements.append(fname)
        return

    def _compare_folders(self, dir1, dir2):

        comp = Compare(dir1, dir2)

        new_dir = [os.path.join(dir1, item) for item in comp.left_only if valuable_item(item)]

        self._new_dir.extend(new_dir)

        diff_files = [os.path.join(dir1, item) for item in comp.diff_files]

        self._diff_files.extend(diff_files)

        for name in comp.diff_files:
            ldiffer(os.path.join(dir1, name), os.path.join(dir2, name))

        for subdir in comp.common_dirs:
            self._compare_folders(os.path.join(dir1, subdir), os.path.join(dir2, subdir))

        return

    def compare_to_local(self):

        print("Check for update... ")

        self.extract_file_archive()

        self._compare_folders('.', '.temp/')

        temp = []
        for elem in self._additional_elements:
            if elem in self._new_dir:
                temp.append(elem)
            else:
                print("update info: %s is already present in archive: omitted to completion" % elem)
        self._additional_elements = temp

        for item in self._new_dir:
            if item not in self._additional_elements:
                self._omitted_elements.append(item)

        if len(self._additional_elements) > 0:
            print("\n    Added to archived :\n")
            for elem in self._additional_elements:
                print("    %s" % elem)
            print("")

        if len(self._omitted_elements) > 0:
            print("\n    Omitted elements : \n")
            for elem in self._omitted_elements:
                print("    %s" % elem)
            print("")

        if len(self._diff_files) > 0:
            print("\n    Diff files : \n")
            for elem in self._diff_elements:
                print("    %s" % elem)
            print("")
        
    def update_file(self, fname):
        if fname not in self.updated_files:
            print("Adding %s to archive" % fname)
            self.updated_files.append(fname)
        else:
            print("warning : %s already in archive (not add).")
        return
            
    def update_version(self, type_version):        
        
        if type_version == "release":
            self._version[0] += 1
            self._version[1] = 0
            self._version[2] = 0
        elif type_version == "major":
            self._version[1] += 1
            self._version[2] = 0
        elif type_version == "minor":
            self._version[2] += 1
        else:
            print("warning : unknown type version.")
            
        return

    def update_archive(self, type_version):

        self.compare_to_local()

        if len(self._additional_elements) > 0:
            print("Updating archive...")

            self.update_version(type_version)

            str_version = '{}.{}.{}'.format(*self._version)
            new_archive = "data_v" + str_version + ".tar.gz"
            copyfile(self._file_archive, new_archive)

            if os.path.exists('.temp/'):
                rmtree('.temp/')
            with tarfile.open(self._file_archive, 'r') as f:
                print("--> Extract data from previous archive")
                f.extractall(".temp/")

            with tarfile.open(new_archive, 'w:bz2') as f:
                print("--> Create new archive")
                for item in os.listdir('.temp/'):
                    f.add(item)
                for item in self._additional_elements:
                    f.add(item)

            self._file_archive = new_archive

            print("--> Delete temporary data")
            rmtree('.temp/')

            print("New archive %s created" % new_archive)
        else:
            print("Archive %s is already up to date" % self._file_archive)

    def upload_archive(self):
        print("Upload %s to AWS S3" % self._file_archive)
        with open(self._file_archive, 'rb') as data:
            self._bucket.upload_fileobj(data, "demo/"+self._file_archive, ExtraArgs={'ACL':'public-read'})


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

    data = DataBase()

    data.download_file_archive()

    data.add("./Cylinder")
    data.add("./unit_test")
    data.update_archive("minor")

    #data.upload_archive()


if __name__ == "__main__":
    main()





