#!/usr/bin/env bash
set -e

function setup_conda_env {
  echo "Setting up conda environment"
  local environment_file="third-party/doxyconfig/environment.yml"

  if [ "${DOXYCONFIG_DIR}" == "." ]; then
    mkdir -p third-party/doxyconfig
    cp environment.yml $environment_file
    cp -r doxygen-awesome-css third-party/doxyconfig/
  fi

  echo "cat $environment_file"
  cat $environment_file

  echo "conda env create --quiet --name ${READTHEDOCS_VERSION} --file $environment_file"
  conda env create --quiet --name "${READTHEDOCS_VERSION}" --file "$environment_file"
}

function install_icons {
  echo "Downloading LizardByte graphics"
  wget "https://raw.githubusercontent.com/LizardByte/.github/master/branding/logos/favicon.ico" \
    -O "${READTHEDOCS_OUTPUT}lizardbyte.ico"
  wget "https://raw.githubusercontent.com/LizardByte/.github/master/branding/logos/logo-128x128.png" \
    -O "${READTHEDOCS_OUTPUT}lizardbyte.png"
}

function install_node_modules {
  echo "Creating output directories"
  mkdir -p "${READTHEDOCS_OUTPUT}html/assets/fontawesome/css"
  mkdir -p "${READTHEDOCS_OUTPUT}html/assets/fontawesome/js"

  echo "Installing node modules"
  pushd "${DOXYCONFIG_DIR}"
  npm install
  popd

  echo "Copying FontAwesome files"
  cp "${DOXYCONFIG_DIR}/node_modules/@fortawesome/fontawesome-free/css/all.min.css" \
    "${READTHEDOCS_OUTPUT}html/assets/fontawesome/css"
  cp "${DOXYCONFIG_DIR}/node_modules/@fortawesome/fontawesome-free/js/all.min.js" \
    "${READTHEDOCS_OUTPUT}html/assets/fontawesome/js"
  cp -r "${DOXYCONFIG_DIR}/node_modules/@fortawesome/fontawesome-free/webfonts" \
    "${READTHEDOCS_OUTPUT}html/assets/fontawesome/"
}

function merge_doxyconfigs {
  echo "Merging doxygen configs"
  cp "${DOXYCONFIG_DIR}/doxyconfig-Doxyfile" "./docs/"
  cp "${DOXYCONFIG_DIR}/doxyconfig-header.html" "./docs/"
  cp "${DOXYCONFIG_DIR}/doxyconfig.css" "./docs/"
  cp "${DOXYCONFIG_DIR}/doxyconfig-readthedocs-search.js" "./docs/"
  cat "./docs/Doxyfile" >> "./docs/doxyconfig-Doxyfile"
}

function build_docs {
  echo "Building docs"
  pushd docs
  doxygen doxyconfig-Doxyfile
  popd
}

setup_conda_env
install_node_modules
install_icons
merge_doxyconfigs
build_docs
