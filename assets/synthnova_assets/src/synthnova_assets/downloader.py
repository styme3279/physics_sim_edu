import asyncio
import aiohttp
import zipfile
import shutil
from loguru import logger
from pathlib import Path
from typing import Union, List

def get_available_filename(directory: Path, file_name: str) -> str:
    base = file_name.rsplit(".", 1)[0]
    ext = "." + file_name.rsplit(".", 1)[1] if "." in file_name else ""
    candidate = file_name
    i = 1
    while (directory / candidate).exists():
        candidate = f"{base}_{i}{ext}"
        i += 1
    return candidate

async def download_file(session: aiohttp.ClientSession, url: str, download_dir: Path, overwrite: bool) -> None:
    original_file_name = url.split("/")[-1]
    target_path = download_dir / original_file_name

    if target_path.exists():
        if not overwrite:
            logger.warning(f"File '{original_file_name}' already exists. Skipping download.")
            return
        else:
            logger.warning(f"Overwriting existing file '{original_file_name}'.")

    try:
        async with session.get(url) as response:
            response.raise_for_status()
            content = await response.read()
            target_path.write_bytes(content)
        logger.success(f"Downloaded: {original_file_name} -> {target_path.resolve()}")

        if target_path.suffix.lower() == ".zip":
            extract_dir = download_dir
            with zipfile.ZipFile(target_path, 'r') as zip_ref:
                zip_files = zip_ref.namelist()
                conflict = False
                for f in zip_files:
                    if (extract_dir / f).exists():
                        conflict = True
                        break

            if conflict:
                if overwrite:
                    for f in zip_files:
                        fp = extract_dir / f
                        if fp.exists():
                            if fp.is_file():
                                fp.unlink()
                            elif fp.is_dir():
                                shutil.rmtree(fp)
                    logger.warning(f"Existing files overwritten during extraction of '{original_file_name}'.")
                else:
                    logger.warning(f"Extraction skipped: files already exist for '{original_file_name}'. Deleting the zip file.")
                    target_path.unlink()
                    return

            with zipfile.ZipFile(target_path, 'r') as zip_ref:
                zip_ref.extractall(extract_dir)
            target_path.unlink()
            logger.success(f"Extracted and removed zip file: {original_file_name}")

    except Exception as e:
        logger.error(f"Failed to download {url}: {e}")


async def download_files(url_list: list[str], download_dir: Path, overwrite: bool) -> None:
    """
    Download multiple files asynchronously.

    Args:
        url_list (list[str]): List of URLs to download.
        download_dir (Path): Directory to save downloaded files.
    """
    download_dir.mkdir(parents=True, exist_ok=True)
    async with aiohttp.ClientSession() as session:
        tasks = [download_file(session, url.strip(), download_dir, overwrite) for url in url_list]
        await asyncio.gather(*tasks)


def download(download_dir: str, urls: Union[str, List[str]]) -> None:
    """
    Entry point to download files from download URLs copied from the Asset Cloud Platform.

    Args:
        urls (str): Download URLs copied from the Asset Cloud Platform.
        download_dir (str): Path to the directory for saving files.
    """
    if isinstance(urls, str):
        url_list = urls.split(",")
    elif isinstance(urls, list):
        url_list = urls
    else:
        raise TypeError("urls must be a string or a list of strings")
    download_path = Path(download_dir)
    asyncio.run(download_files(url_list, download_path, overwrite=False))

def update(download_dir: str, urls: Union[str, List[str]]) -> None:
    """
    Download and overwrite existing files or directories.
    """
    if isinstance(urls, str):
        url_list = urls.split(",")
    elif isinstance(urls, list):
        url_list = urls
    else:
        raise TypeError("urls must be a string or a list of strings")
    download_path = Path(download_dir)
    asyncio.run(download_files(url_list, download_path, overwrite=True))
