#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import re
import datetime
from subprocess import check_output, CalledProcessError, DEVNULL


def get_last_commit():
    git_command = "git log --first-parent --pretty=format:'%h' -n 1"
    return re.search(r'(\w{7})', check_output(git_command.split(), encoding='utf-8')).group(0).strip()


def get_tag_from_this_commit(commit_string):  # FIXME: what's the behavior if we have more than one tag on the commit ?
    try:
        git_command = "git describe --tags --abbrev=0 --exact-match %s" % commit_string
        return check_output(git_command.split(), encoding='utf-8', stderr=DEVNULL).strip()
    except CalledProcessError:
        return None


def get_last_tag_past_this_commit(commit_string):
    try:
        git_command = "git describe --tags --abbrev=0 --candidate=50 %s" % commit_string
        return check_output(git_command.split(), encoding='utf-8', stderr=DEVNULL).strip()
    except CalledProcessError:
        raise Exception("NO TAG SET INTO THIS REPOSITORY.\n\tPLEASE CONSIDER SETTING AT LEAST ONE!")


def get_date_from_this_commit(commit_string):
    git_command = "git show -s --format='%%ai' --date=local %s" % commit_string
    pattern = r'\d{4}(-\d{2}){2}\s\d{2}(:\d{2}){2}\s[+\-]\d{4}'
    date_string = re.search(pattern,
                            check_output(git_command.split(), encoding='utf-8')
                            ).group(0).strip()

    date_format = '%Y-%m-%d %H:%M:%S %z'
    d = datetime.datetime.strptime(date_string, date_format)
    return d


def get_commit_from_this_tag(tag_string):
    try:
        git_command = "git rev-parse --short %s" % tag_string
        return check_output(git_command.split(), encoding='utf-8', stderr=DEVNULL).strip()
    except CalledProcessError:
        return None

def extract_candidate_version_from_string(string):
    pattern = r'(\d+)\.(\d+)(?:\.(\d+))?(?:-([\w-]+))?'  # https://regex101.com/r/GgrfNB/4
    candidate = re.search(pattern, string)
    if candidate is None:
        raise Exception("The given string \"%s\" has no candidate version number enclosed" % string)
    return candidate.group(0)


def get_author_of_this_commit(commit_string):
    git_command = "git show -s --format='%%cn' %s" % commit_string
    author = check_output(git_command.split(), encoding='utf-8').strip()
    author = author[1:len(author)-1]
    git_command = "git show -s --format='%%ce' %s" % commit_string
    email = check_output(git_command.split(), encoding='utf-8').strip()
    email = email[1:len(email)-1]
    return "%s <%s>" % (author, email)


def extract_version_from_git():

    v = VersionNumber()

    # Get the last commit SHA1
    commit_sha1 = get_last_commit()
    date = get_date_from_this_commit(commit_sha1)

    # Get the current tag or the latest past this commit
    tag = get_tag_from_this_commit(commit_sha1)
    v.this_commit_owns_this_tag = True
    if tag is None:  # the current commit has no tag
        v.this_commit_owns_this_tag = False
        tag = get_last_tag_past_this_commit(commit_sha1)

    # Populate major, minor, patch and meta
    pattern = r'^' \
              r'(?P<MAJOR>\d+)\.(?P<MINOR>\d+)' \
              r'(?:\.(?P<PATCH>\d+))?(?:-(?P<META>[\w]+))?' \
              r'$'  # https://regex101.com/r/JGPX5A/1
    try:
        elements = re.match(pattern, tag).groupdict()
    except AttributeError:
        raise Exception("Tag \"%s\" has not a valid syntax" % tag)

    v.major = int(elements['MAJOR'])
    v.minor = int(elements['MINOR'])

    patch = elements['PATCH']
    if patch is None:
        v.patch = 0
    else:
        v.patch = int(patch)

    meta = elements['META']
    if meta is None:
        meta = ''
    v.meta = meta

    # Is the version a release version, a patched release version or a development version
    if not meta and v.this_commit_owns_this_tag:
        if not patch:
            v.release_type = 'release'
        else:
            v.release_type = 'release with patch'
    else:
        v.release_type = 'development'

    v.date = date
    v.commit = commit_sha1
    v.committer_name = get_author_of_this_commit(commit_sha1)

    return v


class VersionNumber:
    def __init__(self):
        self.major = 0
        self.minor = 1
        self.patch = 0
        self.meta = ''
        self.date = datetime.datetime.now()
        self.this_commit_owns_this_tag = False
        self.commit = ''
        self.release_type = 'development'
        self.committer_name = ''

    def __repr__(self):
        return self.normalized_version()

    def __str__(self):
        return self.normalized_version()

    def __gt__(self, other):
        return self.date > other.date

    def __lt__(self, other):
        return self.date < other.date

    def normalized_version(self):
        version_string = "%s.%s" % (self.major, self.minor)

        patch = self.patch
        if patch > 0:
            version_string += ".%s" % patch

        if not self.this_commit_owns_this_tag:
            # We complete META with our information about date and commit SHA1
            version_string += "-%s%sgit%s" % (self.meta, self.date.strftime('%Y%m%d%H%M%S'), self.commit)

        return version_string

    def cmake_string(self):
        return "%s;%s;%s;%s;%s;%s;%s;%s" % (  # TODO: use the last string formatting rule from python
            self.major,
            self.minor,
            self.patch,
            self.normalized_version(),
            self.commit,
            self.date.ctime(),
            self.committer_name,
            self.release_type
        )

    def json_repr(self):
        import json
        d = {
            'major': self.major,
            'minor': self.minor,
            'patch': self.patch,
            'version': self.normalized_version(),
            'commit': self.commit,
            'committer_name': self.committer_name,
            'date': self.date.ctime(),
            'release_type': self.release_type
        }
        return json.dumps(d, sort_keys=False, indent=2)


if __name__ == '__main__':

    import sys
    try:
        arg = sys.argv[1]
    except IndexError:
        raise Exception("getversion.py must be called with the required representation in [normalized, cmake, json]")

    if arg not in ['normalized', 'n', 'cmake', 'c', 'json', 'j']:
        raise Exception("Valid command line arguments are 'normalized' or 'cmake'")

    version = extract_version_from_git()

    if arg == 'normalized' or arg == 'n':
        print(version.normalized_version())
    elif arg == 'json' or arg == 'j':
        print(version.json_repr())
    elif arg == 'cmake' or arg == 'c':
        print(version.cmake_string())
    else:
        raise Exception("Unknown argument %s" % arg)  # Should never happen...
