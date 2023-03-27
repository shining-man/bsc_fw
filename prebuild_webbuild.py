import glob
import shutil
import gzip
import os
Import('env')

env.Execute("$PYTHONEXE -m pip install -v htmlmin")
env.Execute("$PYTHONEXE -m pip install --upgrade htmlmin")


def genDataFiles():
    filetypesToMin =  ['htm', 'css'] 
    filetypesToGzip = ['js', 'ico']

    dataPath = os.path.join(env.get('PROJECT_DIR'), 'data')
    dataSrcPath = os.path.join(env.get('PROJECT_DIR'), 'data_src')

    if(os.path.exists(dataPath)):
        shutil.rmtree(dataPath)

    os.mkdir(dataPath)

    filesToGzip = []
    for extension in filetypesToGzip:
        filesToGzip.extend(glob.glob(os.path.join(dataSrcPath, '*.' + extension)))
    #print('  files to gzip: ' + str(filesToGzip))

    filesToMin = []
    for extension in filetypesToMin:
        filesToMin.extend(glob.glob(os.path.join(dataSrcPath, '*.' + extension)))
    #print('  files to minify: ' + str(filesToMin))

    allFiles = glob.glob(os.path.join(dataSrcPath, '*.*'))
    filesToCopy = list(set(allFiles) - set(filesToGzip) - set(filesToMin))
    #print('  files to copy: ' + str(filesToCopy))

    for file in filesToCopy:
        #print('  Copying file: ' + file + ' to data dir')
        shutil.copy(file, dataPath)

    for file in filesToGzip:
        #print('  GZipping file: ' + file + ' to data dir')
        with open(file, 'rb') as f_in, gzip.open(os.path.join(dataPath, os.path.basename(file) + '.gz'), 'wb') as f_out:
            shutil.copyfileobj(f_in, f_out)

    for file in filesToMin:
        #print('  Minify file: ' + file + ' to data dir')
        shutil.copy(file, os.path.join(dataPath, os.path.basename(file)))
        env.Execute("htmlmin  --keep-optional-attribute-quotes "+file+" "+os.path.join(dataPath, os.path.basename(file)))



#def get_build_flag_value(flag_name):
#    build_flags = env.ParseFlags(env['BUILD_FLAGS'])
#    flags_with_value_list = [build_flag for build_flag in build_flags.get(
#        'CPPDEFINES') if type(build_flag) == list]
#    defines = {k: v for (k, v) in flags_with_value_list}
#    return str(defines.get(flag_name))


genDataFiles()
