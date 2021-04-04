#!/usr/bin/python3
import os, sys, copy

#configs

GLOBAL_REPO_LOCATION = "git@github.com"
LOCAL_REPO_LOCATION = "git@gitnuc"
CREATE_NEW_REMOTE_FOLDERS = False
PUSH_TO_NEW_SERVER = False
DEBUG = False

#some constants
REPO_URL_KEY = "repo"
GITMAN_REPOS_LOCATION = ".gitman/"
GITMAN_FILE=".gitman.yml"

def parse_key_value(line):
  splited = line.split(":", 1)
  if len(splited) == 2:
    return splited
  else:
    return None,None

def load_repos():
  repos = {}
  actual_repo = None
  with open(GITMAN_FILE,'r') as f:
    for line in f:
      line_striped = line.replace(" ","").rstrip()

      if "-name:" in line_striped:
        key,value = parse_key_value(line_striped)
        #print(key,value)
        actual_repo = value
        repos[actual_repo] = {}
      elif actual_repo is not None:
        key,value = parse_key_value(line_striped)
        if key is not None:
          #print(key,value)
          repos[actual_repo][key]=value
  return repos

def filter_old_repos(repos,old_location):
  our_repos = {}
  for repo_name in repos:
    if old_location in repos[repo_name][REPO_URL_KEY]:
      #print(repos[repo_name][REPO_KEY])
      our_repos[repo_name] = copy.deepcopy(repos[repo_name])
  return our_repos

def ssh_run(server,command,debug=True):
  """run remote commands over ssh"""
  to_run = "ssh "+server+" '"+command+"'"
  if debug:
    print(to_run)
  result = os.popen(to_run).read()
  if debug:
    print(result)
  return result

def run_local(command,debug=True):
  """run local commands"""
  if debug:
    print(command)
  result = os.popen(command).read()
  if debug:
    print(result)
  return result

def change_repo_from_to(repo_name,repo,old_location,new_location):
  """change repo from old location to new location,
  create bare repo if CREATE_NEW_REMOTE_FOLDERS,
  and set its new origin and push it"""
  new_repo = copy.deepcopy(repo)
  if DEBUG:
    print("changing repo",repo_name)

  new_repo_link = repo[REPO_URL_KEY].replace(old_location,new_location)
  directory = new_repo_link.replace(new_location,"").replace(":","")
  if DEBUG:
    print(directory,new_repo_link)

  if CREATE_NEW_REMOTE_FOLDERS:
    #create new remote bare repo
    ssh_run(new_location, "mkdir -p "+directory,debug=DEBUG)
    ssh_run(new_location, "cd "+directory+" ; git init --bare",debug=DEBUG)
  #change repo remote
  run_local("cd "+GITMAN_REPOS_LOCATION+repo_name+"; git remote set-url origin "+new_repo_link,debug=DEBUG)
  if PUSH_TO_NEW_SERVER:
    run_local("cd "+GITMAN_REPOS_LOCATION+repo_name+"; git push --all origin",debug=DEBUG)

  #change link in the remote
  new_repo[REPO_URL_KEY] = new_repo_link
  return new_repo

def change_gitman_file(new_file_name,old_repos,new_repos):
  """create new_file_name from GITMAN_FILE with updated
  repo urls from old_repos to new_repos"""
  changing_repo = None
  with open(GITMAN_FILE,'r') as old_file:
    with open(new_file_name,'w') as new_file:
      for line in old_file:
        line_striped = line.replace(" ","").rstrip()
        #print(line_striped)
        if "-name:" in line_striped:
          key,value = parse_key_value(line_striped)

          if new_repos.get(value) is not None:
            #contains specific repo
            changing_repo = value
          else:
            #not repo for change
            changing_repo = None
        elif "repo:" in line_striped and changing_repo is not None:
          #change repo link
          old_link = old_repos[changing_repo][REPO_URL_KEY]
          new_link = new_repos[changing_repo][REPO_URL_KEY]
          #print("from",old_link,"to",new_link)
          line = line.replace(old_link,new_link)

        #write to file
        new_file.write(line)

def print_help(arguments):
  print("run "+str(arguments[0])+" to change between local/global git servers")
  print("-l change to local server "+LOCAL_REPO_LOCATION+" from global "+GLOBAL_REPO_LOCATION)
  print("-g change to global server "+GLOBAL_REPO_LOCATION+" from local "+LOCAL_REPO_LOCATION)
  print("-c to create new server repos")
  print("-p to push to new repotes")
  print("-d to run in debug mode")

if __name__ == "__main__":

  change_from_repo = GLOBAL_REPO_LOCATION
  change_to_repo = LOCAL_REPO_LOCATION

  #parse arguments
  arguments = sys.argv
  if "-h" in arguments or "--help" in arguments:
    print_help(arguments)
    quit()
  if "-l" in arguments:
    print("changing from global repo "+GLOBAL_REPO_LOCATION+" to local "+LOCAL_REPO_LOCATION)
    change_from_repo = GLOBAL_REPO_LOCATION
    change_to_repo = LOCAL_REPO_LOCATION
  if "-g" in arguments:
    print("changing from local repo "+LOCAL_REPO_LOCATION+" to global "+GLOBAL_REPO_LOCATION)
    change_from_repo = LOCAL_REPO_LOCATION
    change_to_repo = GLOBAL_REPO_LOCATION
  if "-c" in arguments:
    print("will create server git repository")
    CREATE_NEW_REMOTE_FOLDERS = True
  if "-p" in arguments:
    print("will push to new remotes")
    PUSH_TO_NEW_SERVER = True
  if "-d" in arguments:
    print("will run in debug mode")
    DEBUG = True

  #parse repos from gitman.yml and filter changable ones
  all_repos = load_repos()
  old_repos = filter_old_repos(all_repos, change_from_repo)

  #change links and create repos of all changable in gitman.yml
  num_changed = 0
  new_repos = {}
  for repo_name in old_repos:
    print(repo_name,old_repos[repo_name][REPO_URL_KEY])
    new_repo = change_repo_from_to(repo_name,old_repos[repo_name],change_from_repo,change_to_repo)
    new_repos[repo_name] = new_repo
    if old_repos[repo_name][REPO_URL_KEY]!=new_repos[repo_name][REPO_URL_KEY]:
      print(" - changed "+repo_name+" from "+old_repos[repo_name][REPO_URL_KEY]+" to "+new_repos[repo_name][REPO_URL_KEY])
      num_changed += 1


  #change root repo remote
  this_repo_remote = run_local("git remote get-url origin --push",debug=DEBUG).rstrip()
  if DEBUG:
    print("this_repo_remote",this_repo_remote)
  new_repo_link = this_repo_remote.replace(change_from_repo,change_to_repo)
  directory = new_repo_link.replace(change_to_repo,"").replace(":","/")
  if CREATE_NEW_REMOTE_FOLDERS:
    ssh_run(change_to_repo, "mkdir -p "+directory,debug=DEBUG)
    ssh_run(change_to_repo, "cd "+directory+" ; git init --bare",debug=DEBUG)
  run_local("git remote set-url origin "+new_repo_link,debug=DEBUG)
  if PUSH_TO_NEW_SERVER:
    run_local("git push --all origin",debug=DEBUG)
  if this_repo_remote!=new_repo_link:
    print(" - changed root repo from "+change_from_repo+" to "+change_to_repo)
    num_changed += 1

  print("changed "+str(num_changed)+" repos")

  if num_changed > 0:
    #copy gitman file to old
    NEW_GITMAN_FILE = GITMAN_FILE+".new"
    OLD_GITMAN_FILE = GITMAN_FILE+".old"

    change_gitman_file(NEW_GITMAN_FILE,old_repos,new_repos)
    run_local("mv "+GITMAN_FILE+" "+OLD_GITMAN_FILE,debug=DEBUG)
    run_local("mv "+NEW_GITMAN_FILE+" "+GITMAN_FILE,debug=DEBUG)
