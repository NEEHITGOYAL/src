version=''
commithash='5b118836238efb9fedc9a951cd02736983b37917'
gittag_short=''
gittag_long='5b11883-dirty'
git_lastmod='Mon, 25 Jan 2021 12:57:57 +0530'
github_url='https://github.com/plasmodic/ecto'

breathe_default_project = 'ecto'
breathe_projects = dict(ecto='/home/narayan/catkin_ws/build/ecto/doc/../api/xml')

# for debug: this is only if you build everything locally
#ecto_module_url_root = '/home/narayan/catkin_ws/build/ecto/doc/../../doc/html/'
# for release
ecto_module_url_root = 'http://plasmodic.github.com/'

intersphinx_mapping = {
                       'ectoimagepipeline': (ecto_module_url_root + 'ecto_image_pipeline', None),
                       'ectoopenni': (ecto_module_url_root + 'ecto_openni', None),
                       'ectoopencv': (ecto_module_url_root + 'ecto_opencv', None),
                       'ectopcl': (ecto_module_url_root + 'ecto_pcl', None),
                       'ectoros': (ecto_module_url_root + 'ecto_ros', None),
                       }

programoutput_path = ''.split(';')
