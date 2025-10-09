pr() {
  local char="$1"
  local count="$2"
  printf "%${count}s" "" | tr ' ' "$char"
}

reset=false
clean=false
verbose=false
upload=false

while getopts "rcvu" opt; do
  case $opt in
    r)
      reset=true
      ;;
    c)
      clean=true
      ;;
    v)
      verbose=true
      ;;
    u)
      upload=true
      ;;
    \?)
      echo "Invalid option: -$OPTARG" >&2
      ;;
  esac
done

if $reset; then
  reset
  echo "Reset terminal"
  source ~/.zshrc
  echo "Sourced .zshrc\n"
fi

if $clean; then
  make clean
  echo "Cleaned project\n"
fi

echo "Compiling project\n"
time make -j$(($(sysctl -n hw.ncpu) + 1)) 2>/dev/null

if [ $? -ne 0 ]; then
    printf "\n\e[41m ┏%s┓ \e[0m\n\e[41m ┃%s┃ \e[0m\n\e[41m ┃%s\e[1;37m!! MAKE FAILED !!\e[0;37m\e[41m%s┃ \e[0m\n\e[41m ┃%s┃ \e[0m\n\e[41m ┗%s┛ \e[0m" \
    "$(pr '━' 76)" "$(pr ' ' 76)" "$(pr ' ' 29)" "$(pr ' ' 30)" "$(pr ' ' 76)" "$(pr '━' 76)"

    upload=false
else
    echo "compiled on $(date '+%Y-%m-%d %H:%M:%S')"
fi

if $upload; then
  pros u
fi