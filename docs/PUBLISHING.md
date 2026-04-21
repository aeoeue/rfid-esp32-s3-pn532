# Publishing Checklist

## Push to GitHub

From inside this folder:

```bash
git init
git add .
git commit -m "Initial PN532 release v0.5.0"
git branch -M main
git remote add origin <your-repo-url>
git push -u origin main
```

## Optional GitHub Release

Suggested tag:

- `v0.5.0`

Suggested release assets:

- `ota/rfid-esp32s3-latest.bin`
- `ota/rfid-esp32s3-0.5.0-20260421-132735.bin`
- `web-installer/firmware/merged.bin`
