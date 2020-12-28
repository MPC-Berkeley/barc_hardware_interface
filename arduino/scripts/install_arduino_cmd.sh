sudo python2 -m pip install setuptools configobj glob2 jinja2 ordereddict

# install command-line arduino tool
if ! hash ano 2>/dev/null; then
    # download tool
    echo "installing arduino command line tool"
    cd ~; git clone https://github.com/scottdarch/Arturo
    cd Arturo

    # install tool
    sudo make install; cd; sudo rm -rf Arturo
    cd;
fi
