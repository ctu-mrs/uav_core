PNAME=$( ps -p "$$" -o comm= )
SNAME=$( echo "$SHELL" | grep -Eo '[^/]+/?$' )
if [ "$PNAME" != "$SNAME" ]; then
  exec "$SHELL" -i "$0" "$@"
  exit "$?"
else
  case $- in
    *i*) ;;
    *)
      exec "$SHELL" -i "$0" "$@"
      exit "$?"
      ;;
  esac
  source ~/."$SNAME"rc
fi

RCFILE=~/."$SNAME"rc

num=`cat $RCFILE | grep "^export $1" | wc -l`

if [ "$num" -lt "1" ]; then
  
  if [ -z "$3" ]; then
    COMMENTARY=""
  else
    COMMENTARY="# $3"
  fi

  echo "export $1=\"$2\" $COMMENTARY" >> $RCFILE
  echo "$2"
else
  var_name=`eval echo $1`
  var_value=`eval echo -e "\\$${var_name}"`
  echo "$var_value"
fi
