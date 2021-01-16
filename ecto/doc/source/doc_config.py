version=''
commithash='c34d9d78b633311d31525fdf80190b37714b3948'
gittag_short=''
gittag_long='c34d9d7-dirty'
git_lastmod='Thu, 14 Jan 2021 22:34:50 +0530'
github_url='https://github.com/plasmodic/ecto'

breathe_default_project = 'ecto'
breathe_projects = dict(ecto='/home/neehit/catkin_ws/build/ecto/doc/../api/xml')

# for debug: this is only if you build everything locally
#ecto_module_url_root = '/home/neehit/catkin_ws/build/ecto/doc/../../doc/html/'
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
