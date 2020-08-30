% Program to read in all the RGB color images in a folder and display the histograms of each color channel.
function RGB_Histogram_Demo()

% Change the current folder to the folder of this m-file.
if(~isdeployed)
	cd(fileparts(which(mfilename)));
end
clc;	% Clear command window.
close all;	% Close all figure windows except those created by imtool.
workspace;	% Make sure the workspace panel is showing.
fontSize = 16;

try
% Read in standard MATLAB color demo images.
% Construct the folder name where the demo images live.
imagesFolder = '/1/';%fileparts(which('cameraman.tif')); % Determine where demo folder is (works with all versions).
if ~exist(imagesFolder, 'dir')
	% That folder didn't exist.  Ask user to specify folder.
 	message = sprintf('Please browse to your image folder');
	button = questdlg(message, 'Specify Folder', 'OK', 'Cancel', 'OK');
	drawnow;	% Refresh screen to get rid of dialog box remnants.
	if strcmpi(button, 'Cancel')
	   return;
	else
		imagesFolder = uigetdir();
		if imagesFolder == 0
			% Exit if uer clicked Cancel.
			return;
		end
	end
end

% Read the directory to get a list of images.
filePattern = [imagesFolder, '\*.jpg'];
jpegFiles = dir(filePattern);
filePattern = [imagesFolder, '\*.tif'];
tifFiles = dir(filePattern);
filePattern = [imagesFolder, '\*.png'];
pngFiles = dir(filePattern);
filePattern = [imagesFolder, '\*.bmp'];
bmpFiles = dir(filePattern);
% Add more extensions if you need to.
imageFiles = [jpegFiles; tifFiles; pngFiles; bmpFiles];

% Bail out if there aren't any images in that folder.
numberOfImagesProcessed = 0;
numberOfImagesToProcess = length(imageFiles);
if numberOfImagesToProcess <= 0
 	message = sprintf('I did not find any JPG, TIF, PNG, or BMP images in the folder\n%s\nClick OK to Exit.', imagesFolder);
	uiwait(msgbox(message));
	return;
end

% Create a figure for our images.
figure;
set(gcf, 'Position', get(0,'Screensize')); % Maximize figure.
set(gcf,'name','Image Analysis Demo','numbertitle','off') 

% Preallocate arrays to hold the mean intensity values of all the images.
HueChannel_Mean = zeros(numberOfImagesToProcess, 1);
SatChannel_Mean = zeros(numberOfImagesToProcess, 1);
ValChannel_Mean = zeros(numberOfImagesToProcess, 1);

% We'll be skipping monochrome and indexed images
% and just looking at true color images.
% Keep track of how many we actually look at.
numberOfImagesToProcess2 = numberOfImagesToProcess;

% Loop though all images, calculating and displaying the histograms.
% and then getting the means of the Hue, Sat, and Val channels.
for k = 1 : numberOfImagesToProcess
	% Read in this one file.
	baseFileName = imageFiles(k).name;
	fullFileName = fullfile(imagesFolder, baseFileName);
	rgb1 = imread(fullFileName);
    rgbImage = rgb2hsv(rgb1);
	[rows, columns, numberOfColorBands] = size(rgbImage);

	% Check to see that it is a color image (3 dimensions).
	% Skip it if it is not true RGB color.
	if numberOfColorBands < 3
		% Skip monochrome or indexed images.
		fprintf('Skipped %s.  It is a grayscale or indexed image.\n', baseFileName);
		% Decrement the number of images that we'll report that we need to look at.
		numberOfImagesToProcess2 = numberOfImagesToProcess2 - 1;  
		continue;
	end
	
	% If we get to here, it's a true color image.
	subplot(3, 3, 1);
	imshow(rgb1, []);
	
	% Create a title for the image.
	caption = sprintf('Original Color Image\n%s\n%d rows by %d columns by %d color channels', ...
		baseFileName, rows, columns, numberOfColorBands);
	% If there are underlines in the name, title() converts the next character to a subscript.
	% To avoid this, replace underlines by spaces.
	caption = strrep(caption, '_', ' ');
	title(caption, 'FontSize', fontSize);
	drawnow;  % Force it to update, otherwise it waits until after the conversion to double.
	
	% Extract the individual Hue, Sat, and Val color channels.
	HueChannel = rgbImage(:, :, 1);
	SatChannel = rgbImage(:, :, 2);
	ValChannel = rgbImage(:, :, 3);
	
	% Hue image:
	subplot(3, 3, 4);
	imshow(HueChannel, []); % Display the image.
	% Compute mean
	HueChannel_Mean(k) = mean(HueChannel(:));
	caption = sprintf('Hue Image.  Mean = %6.2f', HueChannel_Mean(k));
	title(caption, 'FontSize', fontSize);
	% Compute and display the histogram for the Hue image.
	pixelCountHue = PlotHistogramOfOneColorChannel(HueChannel, 7, 'Histogram of Hue Image', 'r');
	drawnow;  % Force it to update, otherwise it waits until after the conversion to double.

	% Sat image:
	subplot(3, 3, 5);
	imshow(SatChannel, []); % Display the image.
	% Compute mean
	SatChannel_Mean(k) = mean(SatChannel(:));
	caption = sprintf('Sat Image.  Mean = %6.2f', SatChannel_Mean(k));
	title(caption, 'FontSize', fontSize);
	% Compute and display the histogram for the Sat image.
	pixelCountSat = PlotHistogramOfOneColorChannel(SatChannel, 8, 'Histogram of Sat Image', 'g');
	drawnow;  % Force it to update, otherwise it waits until after the conversion to double.

	% Val image:
	subplot(3, 3, 6);
	imshow(ValChannel, []); % Display the image.
	numberOfImagesProcessed = numberOfImagesProcessed + 1;
	% Compute mean
	ValChannel_Mean(k) = mean(ValChannel(:));
	caption = sprintf('Val Image.  Mean = %6.2f', ValChannel_Mean(k));
	title(caption, 'FontSize', fontSize);
	% Compute and display the histogram for the Val image.
	pixelCountVal = PlotHistogramOfOneColorChannel(ValChannel, 9, 'Histogram of Val Image', 'b');
	drawnow;  % Force it to update, otherwise it waits until after the conversion to double.
	
	% Plot all three histograms on the same plot.
	subplot(3, 3, 2:3);
	lineWidth = 2;
	hold off;
	plot(pixelCountHue, 'r', 'LineWidth', lineWidth);
	hold on;
	grid on;
	plot(pixelCountSat, 'g', 'LineWidth', lineWidth);
	plot(pixelCountVal, 'b', 'LineWidth', lineWidth);
	title('All the Color Histograms (Superimposed)', 'FontSize', fontSize);
	% Set the x axis range manually to be 0-255.
	xlim([0 255]); 
	
	% Prompt user to continue, unless they're at the last image.
	if k < numberOfImagesToProcess
		promptMessage = sprintf('Currently displaying image #%d of a possible %d:\n%s\n\nDo you want to\nContinue processing, or\nCancel processing?',...
			numberOfImagesProcessed, numberOfImagesToProcess2, baseFileName);
		button = questdlg(promptMessage, 'Continue?', 'Continue', 'Cancel', 'Continue');
		if strcmp(button, 'Cancel')
			break;
		end
	end
end

% Crop off any unassigned values:
HueChannel_Mean = HueChannel_Mean(1:numberOfImagesProcessed);
SatChannel_Mean = SatChannel_Mean(1:numberOfImagesProcessed);
ValChannel_Mean = ValChannel_Mean(1:numberOfImagesProcessed);

% Print to command window
fprintf(1, '                Filename,   Hue Mean, Sat Mean, Val Mean\n');
for k = 1 : length(HueChannel_Mean)
	baseFileName = imageFiles(k).name;
	fprintf(1, '%24s    %6.2f,   %6.2f,     %6.2f\n', ...
		baseFileName, HueChannel_Mean(k), SatChannel_Mean(k), ValChannel_Mean(k));
end

if numberOfImagesProcessed == 1
	caption = sprintf('Done with demo!\n\nProcessed 1 image.\nCheck out the command window for the results');
else
	caption = sprintf('Done with demo!\n\nProcessed %d images.\nCheck out the command window for the results', numberOfImagesProcessed);
end
msgbox(caption);
catch ME
	errorMessage = sprintf('Error in function RGB_Hist_Demo.\n.\n\nError Message:\n%s', ME.message);
	uiwait(warndlg(errorMessage));
end

%----------------------------------------------------------
% Plots a bar chart of the histogram of the color channel.
function pixelCount = PlotHistogramOfOneColorChannel(oneColorChannel, subplotNumber, caption, color)
	try
		% Let's get its histogram into 256 bins.
		[pixelCount, grayLevels] = imhist(oneColorChannel, 256);

		subplot(3, 3, subplotNumber);
		bar(grayLevels, pixelCount, 'FaceColor', color, 'BarWidth', 1); 
		title(caption, 'FontSize', 16);
		grid on;
		% Set the x axis range manually to be 0-255.
		xlim([0 255]); 
	catch ME
		errorMessage = sprintf('Error in function PlotHistogramOfOneColorChannel.\n.\n\nError Message:\n%s', ME.message);
		uiwait(warndlg(errorMessage));
	end
	return;