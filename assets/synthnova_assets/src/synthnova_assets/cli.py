import argparse
from loguru import logger
from synthnova_assets.downloader import download, update

def main():
    parser = argparse.ArgumentParser(description="Synthnova Assets Downloader CLI")
    subparsers = parser.add_subparsers(dest="command", required=True)

    download_parser = subparsers.add_parser("download", help="Download assets without overwriting existing files")
    download_parser.add_argument(
        "-o", "--output_path",
        required=True,
        help="Destination folder path for saving downloaded files"
    )
    download_parser.add_argument(
        "-u", "--urls",
        required=True,
        help="Download URLs copied from the Asset Cloud Platform"
    )

    update_parser = subparsers.add_parser("update", help="Download and overwrite existing assets")
    update_parser.add_argument(
        "-o", "--output_path",
        required=True,
        help="Destination folder path for saving downloaded files"
    )
    update_parser.add_argument(
        "-u", "--urls",
        required=True,
        help="Download URLs copied from the Asset Cloud Platform"
    )

    args = parser.parse_args()

    logger.remove()
    logger.add(lambda msg: print(msg, end=""), colorize=True, format="<level>{level}: {message}</level>")

    if args.command == "download":
        download(args.output_path, args.urls)
    elif args.command == "update":
        update(args.output_path, args.urls)

if __name__ == "__main__":
    main()
