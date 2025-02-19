#!/usr/bin/env python3

# Copyright (c) 2024  Carnegie Mellon University
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import argparse
import subprocess
import sys
import yaml


# load plugins.yaml file
def load_plugins(custom_yaml=None):
    print_blue("  Reading plugins.yaml")
    with open('plugins.yaml', 'r') as f:
        plugins = yaml.load(f, Loader=yaml.FullLoader)

    if custom_yaml:
        print_blue(f"  Reading {custom_yaml} to overwrite plugins.yaml")
        with open(custom_yaml, 'r') as f:
            custom_plugins = yaml.load(f, Loader=yaml.FullLoader)

        for key, value in custom_plugins.items():
            if key == "plugins":
                for key, value in custom_plugins["plugins"].items():
                    if key in plugins["plugins"]:
                        print(f"    Overwriting plugin {key}")
                    else:
                        print(f"    Adding plugin {key}")
                    plugins["plugins"][key] = value
            else:
                if key in plugins:
                    print(f"    Overwriting model {key}")
                else:
                    print(f"    Adding model {key}")
                plugins[key] = value

    return plugins


# func to print text in blue color
def print_blue(text, file=sys.stdout):
    print("\033[94m{}\033[0m".format(text), file=file)


# func to print text in red color
def print_red(text, file=sys.stdout):
    print("\033[91m{}\033[0m".format(text), file=file)


# main function
def main(cabot_model, custom_yaml=None):
    print_blue(f"Reading config for Cabot model {cabot_model}")
    config = load_plugins(custom_yaml)
    if "plugins" in config:
        plugins = config["plugins"]
    else:
        print("  No plugins found", file=sys.stderr)

    # print available plugins in blue color
    print_blue("  Available plugins:")
    for plugin in plugins:
        print(f"    {plugin}")

    build_plugins = []
    model_config = None
    if cabot_model:
        if cabot_model in config:
            model_config = config[cabot_model]

    if not model_config:
        print_red(F"Cannot find config for {cabot_model}")
        sys.exit(1)

    build_plugins = model_config["plugins"] if "plugins" in model_config else []
    default_environment = model_config["environment"] if "environment" in model_config else {}
    networks = model_config["networks"] if "networks" in model_config else {}

    print_blue(f"  {cabot_model} plugins:")
    for plugin in build_plugins:
        print(f"    {plugin}")
    print_blue(f"  {cabot_model} default environment:")
    for (key, value) in default_environment.items():
        print(f"    {key}: {value}")

    if not build_plugins:
        print_red(f"  No plugins found for model {cabot_model}", file=sys.stderr)
        sys.exit(1)

    # execute docker compose with plugin configuration
    print_blue(f"Building plugins for model {cabot_model}")
    merged_services = {}
    merged_config = {
        "services": merged_services,
        "networks": networks
    }
    for build_plugin in build_plugins:
        plugin_config = plugins[build_plugin]
        dockerfile = "docker-compose.yaml"
        if 'dockerfile' in plugin_config:
            dockerfile = plugin_config['dockerfile']
        command = ["docker", "compose", "-f", f"{plugin_config['path']}/{dockerfile}"]
        if 'profile' in plugin_config and plugin_config['profile']:
            command.extend(["--profile", plugin_config["profile"]])
        command.extend(["config", plugin_config["service"]])
        print(f"  {build_plugin}")
        print(f"    config : {plugins[build_plugin]}")
        print(f"    command: {command}")

        # execute docker compose command
        result = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        if result.returncode != 0:
            print_red(f"Failed to build plugin {build_plugin} {result.returncode}", file=sys.stderr)
            print_red(result.stderr, file=sys.stderr)
            sys.exit(1)
        print_blue("    docker compose config success")
        # parse the result of the subprocess.run as yaml
        result_yaml = yaml.safe_load(result.stdout)

        # merge services dict
        if 'services' in result_yaml:
            # iterate result_yaml['services'] dict and add build_plugin as the prefix of the key
            # prefixed_services = {f"{build_plugin}_{key}": value for key, value in result_yaml['services'].items()}
            prefixed_services = {f"{key}": value for key, value in result_yaml['services'].items()}
            # remove profiles from prefixed_services
            for key in prefixed_services:
                if "profiles" in prefixed_services[key]:
                    del prefixed_services[key]["profiles"]
                if "env_file" in prefixed_services[key]:
                    prefixed_services[key]["env_file"].append(".env.default")
                else:
                    prefixed_services[key]["env_file"] = [".env.default"]

            merged_services.update(prefixed_services)

    print_blue("Write merged services into docker-compose-plugins.yaml")
    # output merged_config to docker-compose-plugins.yaml
    with open('docker-compose-plugins.yaml', 'w') as f:
        yaml.dump(merged_config, f, default_flow_style=False)

    with open(".env.default", "w") as f:
        for (key, value) in default_environment.items():
            print(F"{key}=\"{value}\"", file=f)


if __name__ == '__main__':
    def parse_args():
        parser = argparse.ArgumentParser(description='Process some integers.')
        parser.add_argument('-m', '--model', type=str, help='Specify the cabot_model to select plugin list')
        parser.add_argument('-c', '--custom', type=str, help='Specify custom.yaml to override default plugins.yaml')
        return parser.parse_args()

    args = parse_args()
    cabot_model = args.model
    custom_yaml = args.custom

    main(cabot_model, custom_yaml)
