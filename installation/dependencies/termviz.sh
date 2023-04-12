# get the path to this script
MY_PATH=`dirname "$0"`
MY_PATH=`( cd "$MY_PATH" && pwd )`

sudo apt-get -y install cargo

cd $MY_PATH/../../utils/termviz
cargo build --release

ln -s $MY_PATH/../../utils/termviz/target/release/termviz /usr/local/bin/
