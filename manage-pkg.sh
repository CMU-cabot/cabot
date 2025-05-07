#!/usr/bin/env bash

# Load environment variables from .env file
if [ -f .env ]; then
    source .env
fi

# Help function to show usage
usage() {
    echo "Usage: $0 [-R] [-r <repo> [-l] [-d [<version>]] [-v <version>] [-o <output_dir>] [-u]]"
    echo "  -R               Make cabot release zip"
    echo "  -r <repo>        Specify a GitHub repository (e.g., user/repo)"
    echo "  -c               Check what version if used"
    echo "  -l               List all releases and attachments"
    echo "  -v <version>     Check if the specified version is available and list its assets"
    echo "  -d               Download all assets for the specified version or latest if no version is given"
    echo "  -o <output_dir>  Specify an output directory for downloaded files (default: CABOT_SITE_PKG_DIR or ./cabot-navigation/cabot_site_pkg)"
    echo "  -u               Unzip downloaded zip files"
    echo "  -p <tag>         Pull docker images"
    exit 1
}

# Variables
RELEASE=false
REPO=""
CHECK=false
LIST=false
DOWNLOAD=false
VERSION=""
OUTPUT_DIR=${CABOT_SITE_PKG_DIR:-./cabot-navigation/cabot_site_pkg}
UNZIP=false
AUTH_HEADER=()
PULL=

# Check if GITHUB_TOKEN is set
if [ -n "$GITHUB_TOKEN" ]; then
    AUTH_HEADER=(-H "Authorization: token $GITHUB_TOKEN")
fi

# Parse options
while getopts "Rr:cldv:o:up:" opt; do
    case ${opt} in
        R )
            RELEASE=true
            ;;
        r )
            REPO=${OPTARG}
            ;;
        c )
            CHECK=true
            ;;
        l )
            LIST=true
            ;;
        d )
            DOWNLOAD=true
            ;;
        v )
            VERSION=${OPTARG}
            ;;
        o )
            OUTPUT_DIR=${OPTARG}
            ;;
        u )
            UNZIP=true
            ;;
        p )
            PULL=${OPTARG}
            ;;
        * )
            usage
            ;;
    esac
done

if [ -n "$PULL" ]; then
    export CABOT_LAUNCH_IMAGE_TAG=$PULL
    docker compose --profile build pull
    exit 0
fi

# Creates a zip file containing only the minimal files required to run cabot with built docker images
# It is intended to be called from GitHub Actions like `./manage-pkg.sh -R -v {{ github.ref_name }}`.
if [ "$RELEASE" = true ]; then
    echo "Making cabot release zip file"
    if [ -z "VERSION" ]; then
        echo "Please specify a version string"
        exit 1
    fi
    tmpdir=$(mktemp -d)
    echo "Temporary directory created: $tmpdir"
    cabotdir="cabot-${VERSION}"
    releasedir="$tmpdir/$cabotdir"
    mkdir -p $releasedir

    set -f

    IGNORE_FILE=".releaseignore"

    # Create exclusion list and exception list
    patterns=()

    if [[ -f "$IGNORE_FILE" ]]; then
    while IFS= read -r line || [[ -n "$line" ]]; do
        line=$(echo "$line" | sed -E 's/[[:space:]]*#.*$//')
        # echo "$line"
        # Ignore empty lines and comments
        [[ -z "$line" ]] && continue

        patterns+=("$line")
    done < "$IGNORE_FILE"
    fi

    # Remove files that match the exclusion list
    filtered_files=()
    while read file; do
    exclude=false

    # Check if it matches the exclusion list
    for pattern in "${patterns[@]}"; do
        if [[ "$pattern" =~ ^! ]]; then
            pattern=${pattern:1}
            if [[ "$file" == $pattern ]] || [[ "$file" =~ ^$pattern ]]; then
                exclude=false
            fi
        else
            if [[ "$file" == $pattern ]] || [[ "$file" =~ ^$pattern ]]; then
                exclude=true
            fi
        fi
    done

    # Add if not an exclusion target or included in the exception list
    if [[ "$exclude" == false ]]; then
        filtered_files+=("$file")
        # echo include $file
    else
        # echo exclude $file
        :
    fi
    done < <(find . -type l -o -type f | sed 's|^\./||' | sort)

    # Find and copy files excluding patterns from .releaseignore
    for file in ${filtered_files[@]}; do
        mkdir -p "$releasedir/$(dirname "$file")"
        if [[ "$OSTYPE" == "darwin"* ]]; then
            cp -RP "$file" "$releasedir/$file"
        else
            cp -d "$file" "$releasedir/$file"
        fi
    done
    pushd $tmpdir
    zip -r -y $cabotdir.zip $cabotdir
    popd
    cp $tmpdir/$cabotdir.zip ./
    exit 0
fi

# Check if OUTPUT_DIR exists
if [ ! -d "$OUTPUT_DIR" ]; then
    echo "Error: Output directory does not exist: $OUTPUT_DIR"
    exit 1
fi

# Check if repository is specified
if [ -z "$REPO" ]; then
    echo "Error: Repository is required."
    usage
fi

if [ "$CHECK" = true ]; then
    # check what version is used
    echo "Downloading latest release attachments for $REPO..."
    ASSETS=$(curl -s "${AUTH_HEADER[@]}" "https://api.github.com/repos/$REPO/releases/latest" | jq -cr '.assets[] | {url: .url, name: .name}')

    echo "$ASSETS" | while read -r ASSET; do
        NAME=$(echo "$ASSET" | jq -r '.name')
        # get version in <version> tag in package.xml
        DIR=$(echo "$NAME" | cut -d'-' -f1)
        grep "<version>" $OUTPUT_DIR/$DIR/share/$DIR/package.xml | sed -E 's/.*<version>([^<]+)<\/version>.*/\1/'
    done | uniq
    exit 0
fi

# List releases
if [ "$LIST" = true ]; then
    echo "Fetching releases for $REPO..."
    curl -s "${AUTH_HEADER[@]}" "https://api.github.com/repos/$REPO/releases" | jq '.[] | {tag_name, assets: .assets[].name}'
    exit 0
fi

# Download specified version or latest release
if [ "$DOWNLOAD" = true ]; then
    if [ "$VERSION" = "" ]; then
        echo "Downloading latest release attachments for $REPO..."
        ASSETS=$(curl -s "${AUTH_HEADER[@]}" "https://api.github.com/repos/$REPO/releases/latest" | jq -r '.assets[] | {url: .url, name: .name}')
    else
        echo "Downloading release $VERSION attachments for $REPO..."
        RELEASE=$(curl -s "${AUTH_HEADER[@]}" "https://api.github.com/repos/$REPO/releases/tags/$VERSION")
        if echo "$RELEASE" | jq -e 'has("message")' > /dev/null; then
            echo "Error: Version $VERSION not found."
            exit 1
        fi
        ASSETS=$(echo "$RELEASE" | jq -c '.assets[] | {url: .url, name: .name, size: .size}')
    fi
    
    echo "$ASSETS" | while read -r ASSET; do
        URL=$(echo "$ASSET" | jq -r '.url')
        NAME=$(echo "$ASSET" | jq -r '.name')
        SIZE=$(echo "$ASSET" | jq -r '.size')
        FILE_PATH="$OUTPUT_DIR/$NAME"
        if [[ ! -e $FILE_PATH ]] || [[ $SIZE != $(stat -c%s $FILE_PATH) ]]; then
            echo "Downloading $NAME to $OUTPUT_DIR..."
            curl -L -o "$FILE_PATH" "${AUTH_HEADER[@]}" -H 'Accept: application/octet-stream' "$URL"
        else
            echo "Already $NAME exists in $OUTPUT_DIR"
        fi
        
        # Unzip if the -u option was specified and the file is a zip
        if [ "$UNZIP" = true ] && [[ "$FILE_PATH" == *.zip ]]; then
            # create backup before unzip
            DIR=$(unzip -Z1 $FILE_PATH | cut -d/ -f1 | sort -u)
            if [ "$DIR" = "" ]; then
                echo "Unzip failed. Cannot unzip $FILE_PATH."
                exit 1
            fi
            DIR_PATH=$OUTPUT_DIR/$DIR
            TEMP_PATH=$DIR_PATH.backup
            FAILED_PATH=$DIR_PATH.failed
            if [[ -e $DIR_PATH ]]; then
                echo "Creating a backup of $DIR_PATH..."
                mv $DIR_PATH $TEMP_PATH
            fi
            # unzip
            echo "Unzipping $FILE_PATH..."
            unzip -o "$FILE_PATH" -d "$OUTPUT_DIR"
            # remove or recover backup depending on unzip success/fail
            ret=$?
            if [ $ret -eq 0 ]; then
                if [[ -e "$TEMP_PATH" ]]; then
                    echo "Unzip succeeded. Removing a backup of $DIR_PATH..."
                    rm -rf $TEMP_PATH
                fi
            else
                if [[ -e "$TEMP_PATH" ]]; then
                    echo "Unzip failed. Recovering a backup of $DIR_PATH"
                    mv $DIR_PATH $FAILED_PATH
                    mv $TEMP_PATH $DIR_PATH
                    rm -rf $FAILED_PATH
                else
                    echo "Unzip failed. Removing incomplete $DIR_PATH"
                    rm -rf $DIR_PATH
                fi
            fi
        fi
    done
    exit 0
fi

if [ -n "$VERSION" ]; then    
    echo "Checking if version $VERSION exists for $REPO..."
    RELEASE=$(curl -s "${AUTH_HEADER[@]}" "https://api.github.com/repos/$REPO/releases/tags/$VERSION")
    if echo "$RELEASE" | jq -e 'has("message")' > /dev/null; then
        echo "Error: Version $VERSION not found."
        exit 1
    else
        echo "Version $VERSION is available. Listing assets:"
        echo "$RELEASE" | jq '.assets[].name'
    fi
    ASSETS=$(echo "$RELEASE" | jq -c '.assets[] | {url: .url, name: .name, size: .size}')

    echo "$ASSETS" | while read -r ASSET; do
        NAME=$(echo "$ASSET" | jq -r '.name')
        SIZE=$(echo "$ASSET" | jq -r '.size')
        FILE_PATH="$OUTPUT_DIR/$NAME"

        # Unzip if the -u option was specified and the file is a zip
        if [ "$UNZIP" = true ] && [[ "$FILE_PATH" == *.zip ]] && [[ -e "$FILE_PATH" ]] && [[ $SIZE = $(stat -c%s $FILE_PATH) ]]; then
            # create backup before unzip
            DIR=$(unzip -Z1 $FILE_PATH | cut -d/ -f1 | sort -u)
            if [ "$DIR" = "" ]; then
                echo "Unzip failed. Cannot unzip $FILE_PATH."
                exit 1
            fi
            DIR_PATH=$OUTPUT_DIR/$DIR
            TEMP_PATH=$DIR_PATH.backup
            FAILED_PATH=$DIR_PATH.failed
            if [[ -e $DIR_PATH ]]; then
                echo "Creating a backup of $DIR_PATH..."
                mv $DIR_PATH $TEMP_PATH
            fi
            # unzip
            echo "Unzipping $FILE_PATH..."
            unzip -o "$FILE_PATH" -d "$OUTPUT_DIR"
            # remove or recover backup depending on unzip success/fail
            ret=$?
            if [ $ret -eq 0 ]; then
                if [[ -e "$TEMP_PATH" ]]; then
                    echo "Unzip succeeded. Removing a backup of $DIR_PATH..."
                    rm -rf $TEMP_PATH
                fi
            else
                if [[ -e "$TEMP_PATH" ]]; then
                    echo "Unzip failed. Recovering a backup of $DIR_PATH"
                    mv $DIR_PATH $FAILED_PATH
                    mv $TEMP_PATH $DIR_PATH
                    rm -rf $FAILED_PATH
                else
                    echo "Unzip failed. Removing incomplete $DIR_PATH"
                    rm -rf $DIR_PATH
                fi
            fi
        else
            if [[ -e "$FILE_PATH" ]] && [[ $SIZE != $(stat -c%s $FILE_PATH) ]]; then
                echo "File size of downloaded $NAME is different from the asset in $REPO."
            fi
            echo "Asset $NAME is available for download. Use -d to download."
        fi
    done
    exit 0
fi

usage
exit 1
