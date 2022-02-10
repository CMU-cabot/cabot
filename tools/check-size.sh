#!/bin/bash

reverse() {
    # first argument is the array to reverse
    # second is the output array
    declare -n arr="$1" rev="$2"
    for i in "${arr[@]}"
    do
        rev=("$i" "${rev[@]}")
    done
}

pattern=cabot
verbose=0
pass=
exclude="nvidia/cuda|ubuntu:focal|ros:galactic|nvcr.io/nvidia"

function usage {
    echo "Usage:"
    echo "  $0 -p cabot -P <sudo pass>"
    echo ""
    echo "-h                 show help"
    echo "-P <sudo pass>     specify sudo pass to check layer size"
    echo "-v                 verbose"
    echo "-p <pattern>       image name pattern (default=$pattern)"
    echo "-e <exclude pat>   docker image name pattern to exclude from size (default=$exclude)"
}

while getopts "hp:P:ve:" arg; do
    case $arg in
	h)
	    usage
	    exit
	    ;;
	p)
	    pattern=$OPTARG
	    ;;
	P)
	    pass=$OPTARG
	    ;;
	v)
	    verbose=1
	    ;;
	e)
	    exclude=$OPTARG
	    ;;
    esac
done

declare -A all_layers
declare -A image_layers

readarray -t images_i < <(docker images --format "{{.Repository}}:{{.Tag}}" | grep -E "$pattern")
readarray -t images_e < <(docker images --format "{{.Repository}}:{{.Tag}}" | grep -E "$exclude")

images=( "${images_i[@]}" "${images_e[@]}" )

for image in "${images[@]}"; do
    readarray -t layers < <(docker inspect $image | jq -r .[0].RootFS.Layers[])

    rlayers=()
    reverse layers rlayers
    
    image_layers[$image]="${rlayers[@]}"
    for layer in "${layers[@]}"; do
	all_layers[$layer]=1
    done
done

if [ -z $pass ]; then
    sudo echo ""
else
    echo $pass | sudo -S echo ""
fi

for layer in "${!all_layers[@]}"; do
    readarray -t diff < <(sudo find /var/lib/docker/image/overlay2/layerdb -name "diff" -exec grep -rl $layer {} +)
    dir=`dirname ${diff[0]}`
    size=`sudo cat $dir/size`
    all_layers[$layer]=$size
done


readarray -t sorted < <(for a in "${!image_layers[@]}"; do echo "$a"; done | sort)

for parent in "${sorted[@]}"; do
    players=(${image_layers[$parent]})
    findex=0
    first=${players[$findex]}
    while [ ! -z ${all_layers[$first]} ] && [ ${all_layers[$first]} -lt 100 ]; do
	findex=$(expr $findex + 1)
	first=${players[$findex]}
    done
    if [ -z ${all_layers[$first]} ]; then
	continue
    fi
    
    for child in "${sorted[@]}"; do
	if [ $parent == $child ]; then
	    continue
	fi

	clayers=(${image_layers[$child]})
	temp=()
	for layer in "${clayers[@]}"; do
	    if [ $first == $layer ]; then
		temp+=(${parent})
		#break
	    else 
		temp+=($layer)
	    fi
	done
	image_layers[$child]=${temp[@]}
    done
done

atotal=0
for name in "${sorted[@]}"; do
    if [[ $name =~ $exclude ]]; then
	continue
    fi
    if [[ $name =~ ${prefix}__ ]]; then
	continue
    fi
    if [ $verbose -eq 1 ]; then
	echo ""
    fi
    total=0
    base=
    for layer in ${image_layers[$name]}; do
	if [[ $layer =~ ^sha256:.* ]]; then
	    size=${all_layers[$layer]}
	    total=$(expr $total + $size)
	    if [ $verbose -eq 1 ]; then
		echo "    $layer $size"
	    fi
	else
	    base=$layer
	    if [ $verbose -eq 1 ]; then
		echo "    $layer"
	    fi
	    break
	fi
    done
    atotal=$(expr $atotal + $total)
    total=`echo "scale=2;$total/1024/1024" | bc`
    printf "%8.2f MB: %-80s (parent:%s)\n" $total $name $base
done
echo "----prebuild images----"
for name in "${sorted[@]}"; do
    if [[ $name =~ $exclude ]]; then
	continue
    fi
    if [[ ! $name =~ ${prefix}__ ]]; then
	continue
    fi
    if [ $verbose -eq 1 ]; then
	echo ""
    fi
    total=0
    base=
    for layer in ${image_layers[$name]}; do
	if [[ $layer =~ ^sha256:.* ]]; then
	    size=${all_layers[$layer]}
	    total=$(expr $total + $size)
	    if [ $verbose -eq 1 ]; then
		echo "    $layer $size"
	    fi
	else
	    base=$layer
	    if [ $verbose -eq 1 ]; then
		echo "    $layer"
	    fi
	    break
	fi
    done
    atotal=$(expr $atotal + $total)
    total=`echo "scale=2;$total/1024/1024" | bc`
    printf "%8.2f MB: %-80s (parent:%s)\n" $total $name $base
done
atotal=`echo "scale=2;$atotal/1024/1024" | bc`
printf "%8.2f MB: Total\n" $atotal

echo ""
echo "----------- excluded images -------------"
atotal=0
for name in "${sorted[@]}"; do
    if [[ ! $name =~ $exclude ]]; then
	continue
    fi
    total=0
    for layer in ${image_layers[$name]}; do
	if [[ $layer =~ sha256:.* ]]; then
	    size=${all_layers[$layer]}
	    total=$(expr $total + $size)
	    #echo "    $layer $size"
	fi
    done
    atotal=$(expr $atotal + $total)
    total=`echo "scale=2;$total/1024/1024" | bc`
    printf "%8.2f MB: %s\n" $total $name
done
atotal=`echo "scale=2;$atotal/1024/1024" | bc`
printf "%8.2f MB: Total\n" $atotal
