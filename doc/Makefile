SOURCES = firmware_update.wiki

all: markdown html

markdown: firmware_update.md

html: firmware_update.html

firmware_update.md: firmware_update.wiki
	pandoc --from=mediawiki --to=markdown --output=firmware_update.md firmware_update.wiki

firmware_update.html: firmware_update.wiki
	pandoc --from=mediawiki --to=html --output=firmware_update.html firmware_update.wiki

clean:
	rm -f firmware_update.md firmware_update.html
