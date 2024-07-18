# @copyright Copyright (c) 2022 Gerhard Schubert GmbH - All Rights Reserved
#
# Unauthorized copying of this file, via any medium is strictly prohibited
# Proprietary and confidential

# spd log does not log in chronological order 
# therefore this scipt takes the path to the 
# spdlog file and does an alphanumeric sorting
# and saved the sorted logs to a new file

import fileinput

data = fileinput.FileInput()
sortedLogsFile = open("sortedLogs.log","w")

#read from cmd args
with fileinput.input() as f:
    for line in sorted(f):
        print(line)
        sortedLogsFile.write(line)

sortedLogsFile.close()
